/* 

  GPL LICENSE SUMMARY

  Copyright(c) Pat Brouillette. All rights reserved.

  This program is free software; you can redistribute it and/or modify 
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  General Public License for more details.

  You should have received a copy of the GNU General Public License 
  along with this program; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution 
  in the file called LICENSE.GPL.

  Contact Information:
    Pat Brouillette  pfrench@acm.org

*/
#include "spdifAnalyzer.h"
#include "spdifAnalyzerSettings.h"
#include <AnalyzerChannelData.h>
#include <string.h>

/* add the "C" bitstream parser library as code and set up some "C" callback stubs */
extern "C" {

#include "spdif.c"

static void c_sample_callback( void *userdata, uint64_t t, uint64_t tend, enum SpdifFrameType ft, uint32_t aud_sample )
{
    spdifAnalyzer   *sba = (spdifAnalyzer *)userdata;
    sba->sample_callback(t,tend,ft,aud_sample);
}

static void c_status_callback( void *userdata, uint64_t t, uint64_t tend, struct SpdifChannelStatus *status )
{
    spdifAnalyzer   *sba = (spdifAnalyzer *)userdata;
    sba->status_callback(t,tend,status);
}

};

spdifAnalyzer::spdifAnalyzer()
:   Analyzer2(),
    mSettings( new spdifAnalyzerSettings() ),
    mSimulationInitilized( false ),
    mHasChannelStatus( false ),
    mIecState( IEC61937_IDLE ),
    mIecDataType( 0 ),
    mIecBurstStart( 0 ),
    mIecPaFt( sft_invalid ),
    mIsNonAudio( false ),
    mIecDetected( false ),
    mIecLastType( 0 ),
    mIecPaTend( 0 ),
    mIecPbTend( 0 ),
    mIecPcTend( 0 )
{
    struct SpdifBitstreamCallbacks  cb;

    cb.userdata    = this;
    cb.cb_sample   = c_sample_callback;
    cb.cb_status   = c_status_callback;

    mSba = SpdifBitstreamAnalyzer_Create(&cb);

    memset( &mLastChannelStatus, 0, sizeof(mLastChannelStatus) );

    SetAnalyzerSettings( mSettings.get() );
}

spdifAnalyzer::~spdifAnalyzer()
{
    KillThread();
    SpdifBitstreamAnalyzer_Delete( mSba );
}

void spdifAnalyzer::SetupResults()
{
    mResults.reset( new spdifAnalyzerResults( this, mSettings.get() ) );
    SetAnalyzerResults( mResults.get() );
    mResults->AddChannelBubblesWillAppearOn( mSettings->mInputChannel );
}

void spdifAnalyzer::WorkerThread()
{
    mSampleRateHz = GetSampleRate();

    mSerial = GetAnalyzerChannelData( mSettings->mInputChannel );

    U64 prev_edge = mSerial->GetSampleNumber();

    mPrevSample = mPrevStatus = prev_edge;
    mPrevSampleEnd = mPrevStatusEnd = prev_edge;
    mSamplesSinceLastBSync = 0;

    /* 상태머신 초기화 */
    mIecState      = IEC61937_IDLE;
    mIecDataType   = 0;
    mIecBurstStart = 0;
    mIecPaFt       = sft_invalid;
    mIsNonAudio    = false;
    mIecDetected   = false;
    mIecLastType   = 0;
    mIecPaTend     = 0;
    mIecPbTend     = 0;
    mIecPcTend     = 0;

    SpdifBitstreamAnalyzer_Reset(mSba);

    for( ; ; )
    {
        mSerial->AdvanceToNextEdge();

        U64 cur_edge = mSerial->GetSampleNumber();

        SpdifBitstreamAnalyzer_AddEdge( mSba, cur_edge - prev_edge, mSerial->GetBitState() == BIT_HIGH );

        prev_edge = cur_edge;
    }
}

bool spdifAnalyzer::NeedsRerun()
{
    return false;
}

U32 spdifAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels )
{
    if( mSimulationInitilized == false )
    {
        mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
        mSimulationInitilized = true;
    }

    return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 spdifAnalyzer::GetMinimumSampleRateHz()
{
    return 1000000 * 4;
}

const char* spdifAnalyzer::GetAnalyzerName() const
{
    return "SPDIF " SPDIF_ANALYZER_VERSION;
}

const char* GetAnalyzerName()
{
    return "SPDIF " SPDIF_ANALYZER_VERSION;
}

Analyzer* CreateAnalyzer()
{
    return new spdifAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
    delete analyzer;
}

/* ======================================================================
   sample_callback
   - Validity bit 감지 → PCM / Non-audio 분기
   - IEC 61937 상태머신: Pa→Pb→Pc→Pd 추적
   ====================================================================== */
void spdifAnalyzer::sample_callback( uint64_t t, uint64_t tend,
                                     enum SpdifFrameType ft, uint32_t aud_sample )
{
    /* gap(불연속) 마커
       Non-audio(IEC61937) 모드에서는 0x0000 패딩 구간의
       갭을 에러로 표시하지 않음
       억제 조건:
         1. validity=1 (이 서브프레임이 Non-audio)
         2. mIsNonAudio=true (Channel Status로 확인된 Non-audio 스트림)
            → 한번 true가 되면 sticky: 스트림 리셋(B-sync 이상) 전까지 유지
         3. IEC61937 상태머신이 IDLE이 아님 (Pa 감지 후 ~ Pd 완료 사이)
            → 버스트 내부 패딩 구간 보호 */
    uint32_t validity_check  = (aud_sample >> 28) & 0x1;
    bool is_nonpcm_frame = ( validity_check != 0 )
                        || mIsNonAudio
                        || ( mIecState != IEC61937_IDLE );

    if ( (mPrevSampleEnd != t) && (mPrevSampleEnd != 0) && !is_nonpcm_frame ) {
        Frame eframe;
        eframe.mData1 = t - mPrevSampleEnd;
        eframe.mData2 = 0;
        eframe.mFlags = DISPLAY_AS_ERROR_FLAG;
        eframe.mStartingSampleInclusive = mPrevSampleEnd + 1;
        eframe.mEndingSampleInclusive   = t - 1;
        eframe.mType  = sft_invalid;
        mResults->AddFrame( eframe );
        mResults->AddMarker( mPrevSampleEnd, AnalyzerResults::ErrorX, mSettings->mInputChannel );
    }

    Frame frame;

    mSamplesSinceLastBSync++;

    /* B-sync 마커 */
    if ( sft_B == ft ) {
        if ( 384 == mSamplesSinceLastBSync ) {
            mResults->AddMarker( t, AnalyzerResults::Dot, mSettings->mInputChannel );
        } else {
            mResults->AddMarker( t, AnalyzerResults::ErrorDot, mSettings->mInputChannel );
        }
        mSamplesSinceLastBSync = 0;

        /* E-AC-3는 버스트(6144 서브프레임) 안에 B-sync가 32번 포함됨
           → B-sync마다 상태머신을 리셋하면 Pa→Pb→Pc→Pd 추적이 끊김
           → B-sync 리셋을 제거하고, Pa→Pb 연속성 검증으로 오인식 차단 */
    }

    /* ------------------------------------------------------------------
       Validity bit (bit28) 확인
       0 = Linear PCM 유효 샘플
       1 = Non-audio / Invalid (Dolby, DTS 등)
    ------------------------------------------------------------------ */
    uint32_t validity = (aud_sample >> 28) & 0x1;   /* validity_check와 동일값 */
    uint32_t payload  = (aud_sample >>  4) & 0x00FFFFFF;   /* bit[4-27] 24비트 */

    /* Non-audio 모드 갱신
       validity=1 → 즉시 true (sticky: 한번 Non-audio면 다음 CS 블록 전까지 유지)
       false 전환은 status_callback()에서 CS byte[0] bit[1]=0 확인 시에만 수행
       → 이렇게 해야 CS 블록 도착 전 초기 구간에서도 gap 억제가 동작함 */
    if ( validity ) mIsNonAudio = true;

    if ( validity ) {
        /* Non-audio: 24비트 페이로드 상위 16비트를 mData1에 저장 */
        frame.mData1 = (int64_t)((payload >> 8) & 0xFFFF);
    } else {
        /* Linear PCM: bit[12-27] 를 signed 16bit 로 추출 (기존 공식) */
        frame.mData1 = ((int)aud_sample << 4) >> 16;
    }

    frame.mData2 = aud_sample;   /* raw 32비트는 항상 보존 */

    /* ------------------------------------------------------------------
       IEC 61937 상태머신 (v6 — 연속성 엄격 검증)
       Pa(0xF872) -> Pb(0x4E1F) -> Pc(data-type) -> Pd(length)

       핵심 변경:
       1. Pa→Pb→Pc→Pd 각 단계가 반드시 연속 서브프레임이어야 함
          (tend[n] + 1 == t[n+1] 검증)
          → E-AC-3 데이터 내부의 우연한 0xF872/0x4E1F 패턴 차단
       2. B-sync에서 상태머신 리셋 제거
          → E-AC-3 버스트(6144 서브프레임) 안에 B-sync가 32번 끼어도 추적 유지
       3. in_burst_window 조건 제거 (연속성 검증으로 대체)
       4. Pd 범위 검증: 비표준 값이면 버림
          → 정상 E-AC-3 PayloadBits: 6144~98304 범위
    ------------------------------------------------------------------ */
    uint16_t word16  = (uint16_t)((aud_sample >> 12) & 0xFFFF);
    bool is_m_or_b   = ( ft == sft_M || ft == sft_B );
    bool is_w        = ( ft == sft_W );

    /* Pa→Pb 연속성: Pa의 tend 다음 서브프레임이 Pb여야 함
       SPDIF 서브프레임은 tend+1에서 시작하므로 t == mIecPaTend + 1 */
    bool pa_pb_consecutive = ( t == mIecPaTend + 1 );
    bool pb_pc_consecutive = ( t == mIecPbTend + 1 );
    bool pc_pd_consecutive = ( t == mIecPcTend + 1 );

    switch ( mIecState )
    {
        case IEC61937_IDLE:
            /* Pa는 항상 탐색 (B-sync 윈도우 제거)
               단, ft가 M 또는 B인 서브프레임(Left ch)에서만 */
            if ( 0xF872 == word16 && is_m_or_b ) {
                mIecState      = IEC61937_GOT_PA;
                mIecBurstStart = t;
                mIecPaFt       = ft;
                mIecPaTend     = tend;
            }
            break;

        case IEC61937_GOT_PA:
            if ( 0x4E1F == word16 && is_w && pa_pb_consecutive ) {
                /* Pa 바로 다음 서브프레임에서 Pb 확인 → 진짜 프리앰블 */
                mIecState  = IEC61937_GOT_PB;
                mIecPbTend = tend;
            } else if ( 0xF872 == word16 && is_m_or_b ) {
                /* 새 Pa 발견 → 재시작 */
                mIecBurstStart = t;
                mIecPaFt       = ft;
                mIecPaTend     = tend;
            } else {
                /* Pb가 연속으로 오지 않으면 오인식 → 리셋 */
                mIecState = IEC61937_IDLE;
            }
            break;

        case IEC61937_GOT_PB:
            if ( is_m_or_b && pb_pc_consecutive ) {
                /* Pb 바로 다음 서브프레임에서 Pc 읽기 */
                mIecDataType = (uint8_t)(word16 & 0x1F);
                mIecState    = IEC61937_GOT_PC;
                mIecPcTend   = tend;
            } else {
                mIecState = IEC61937_IDLE;
            }
            break;

        case IEC61937_GOT_PC:
        {
            if ( !is_w || !pc_pd_consecutive ) {
                /* Pd가 연속으로 오지 않으면 오인식 → 리셋 */
                mIecState = IEC61937_IDLE;
                break;
            }

            uint16_t payload_bits = word16;

            /* Pd 범위 검증
               E-AC-3 정상 PayloadBits: 최소 1000bits, 최대 98304bits(6144×16)
               AC-3:    최대 18688bits
               DTS:     최대 32768bits
               범위 밖이면 오인식으로 판단하고 리셋 */
            if ( payload_bits < 512 || payload_bits > 98304 ) {
                mIecState = IEC61937_IDLE;
                break;
            }

            Frame iecFrame;
            iecFrame.mData1 = (uint64_t)mIecDataType;
            iecFrame.mData2 = (uint64_t)payload_bits;
            iecFrame.mFlags = 0;
            iecFrame.mType  = FRAME_TYPE_IEC61937;
            iecFrame.mStartingSampleInclusive = mIecBurstStart;
            iecFrame.mEndingSampleInclusive   = tend;
            mResults->AddFrame( iecFrame );
            mResults->CommitResults();

            mResults->AddMarker( mIecBurstStart, AnalyzerResults::UpArrow,
                                 mSettings->mInputChannel );

            /* IEC61937 감지 이력 기록 (CS 표시 보정용) */
            mIecDetected = true;
            mIecLastType = mIecDataType;
            /* Non-audio sticky 플래그 강제 설정 */
            mIsNonAudio  = true;

            mIecState    = IEC61937_IDLE;
            mIecDataType = 0;
            break;
        }

        default:
            mIecState = IEC61937_IDLE;
            break;
    }

    /* AC-3 sync word 감지 — 기존 로직 유지 (하위 호환) */
    if ( (0xF872 == m_PrevPCM) && (0x4E1F == (uint16_t)frame.mData1) ) {
        m_AC3_Detected++;
    }
    m_PrevPCM = (U16)frame.mData1;

    frame.mFlags = 0;
    frame.mStartingSampleInclusive = t + 1;
    frame.mEndingSampleInclusive   = tend;
    frame.mType  = ft;
    mResults->AddFrame( frame );

    mPrevSample    = t;
    mPrevSampleEnd = tend;
    mResults->CommitResults();
    ReportProgress( t );
}

/* ======================================================================
   status_callback
   - IEC 60958 Channel Status 192비트 파싱
   - byte[0-4] 를 Frame 으로 저장하여 Saleae UI 에 표시
   ====================================================================== */
void spdifAnalyzer::status_callback( uint64_t t, uint64_t tend,
                                     struct SpdifChannelStatus *status )
{
    if ( mPrevStatus ) {
        mResults->CommitPacketAndStartNewPacket();
        mResults->CommitResults();
    }

    /* Channel Status 24바이트 저장 */
    memcpy( &mLastChannelStatus, status, sizeof(mLastChannelStatus) );
    mHasChannelStatus = true;

    /* Channel Status byte[0] bit[1] = Non-audio 플래그
       IEC61937이 한번이라도 감지됐으면 CS가 PCM으로 와도 Non-audio 유지
       → 장치가 CS byte[0]를 잘못 설정하는 경우 보호 */
    if ( status->channel_status_left[0] & 0x02 ) {
        mIsNonAudio = true;   /* CS에서 Non-audio 확인 → true */
    } else if ( !mIecDetected ) {
        mIsNonAudio = false;  /* IEC61937 미감지 상태에서만 false로 내림 */
    }
    /* mIecDetected=true이면 CS가 뭐라고 오든 mIsNonAudio=true 유지 */

    /* ------------------------------------------------------------------
       Channel Status Frame
       mData1 [63:56] = byte[3] sample rate
       mData1 [55:48] = byte[2] source/channel
       mData1 [47:40] = byte[1] category code
       mData1 [39:32] = byte[0] control bits
       mData2 [7:0]   = byte[4] word length
    ------------------------------------------------------------------ */
    Frame csFrame;
    csFrame.mData1 =
        ( (uint64_t)status->channel_status_left[0]        ) |
        ( (uint64_t)status->channel_status_left[1] <<  8  ) |
        ( (uint64_t)status->channel_status_left[2] << 16  ) |
        ( (uint64_t)status->channel_status_left[3] << 24  ) |
        ( (uint64_t)status->channel_status_left[4] << 32  ) |
        ( (uint64_t)status->channel_status_left[5] << 40  );
    csFrame.mData2 =
        ( (uint64_t)status->channel_status_right[0]       ) |
        ( (uint64_t)status->channel_status_right[1] <<  8 ) |
        ( (uint64_t)status->channel_status_right[2] << 16 ) |
        ( (uint64_t)status->channel_status_right[3] << 24 ) |
        ( mIecDetected ? ((uint64_t)1 << 32) : 0 );  /* bit32: IEC61937 감지 이력 */

    csFrame.mFlags = 0;
    csFrame.mType  = FRAME_TYPE_CHANNEL_STATUS;
    csFrame.mStartingSampleInclusive = t;
    csFrame.mEndingSampleInclusive   = tend;
    mResults->AddFrame( csFrame );

    mPrevStatus    = t;
    mPrevStatusEnd = tend;
    mResults->CommitResults();   /* CS Frame을 UI에 반영 */
}
