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
    mIsNonAudio( false )
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
    return "SPDIF";
}

const char* GetAnalyzerName()
{
    return "SPDIF";
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
       판단 기준 1: validity=1 (서브프레임 단위)
       판단 기준 2: Channel Status byte[0] bit[1]=1 (블록 단위, 더 정확) */
    uint32_t validity_check = (aud_sample >> 28) & 0x1;
    bool is_nonpcm_frame = ( validity_check != 0 ) || mIsNonAudio;

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

    /* B-sync 마커
       B-sync가 오면 IEC61937 상태머신도 리셋
       → 버스트 경계가 바뀌면 Pa~Pd 순서가 깨지기 때문 */
    if ( sft_B == ft ) {
        if ( 384 == mSamplesSinceLastBSync ) {
            mResults->AddMarker( t, AnalyzerResults::Dot, mSettings->mInputChannel );
        } else {
            mResults->AddMarker( t, AnalyzerResults::ErrorDot, mSettings->mInputChannel );
        }
        mSamplesSinceLastBSync = 0;

        /* B-sync 경계에서 상태머신 리셋 */
        if ( mIecState != IEC61937_IDLE ) {
            mIecState    = IEC61937_IDLE;
            mIecDataType = 0;
        }
    }

    /* ------------------------------------------------------------------
       Validity bit (bit28) 확인
       0 = Linear PCM 유효 샘플
       1 = Non-audio / Invalid (Dolby, DTS 등)
    ------------------------------------------------------------------ */
    uint32_t validity = (aud_sample >> 28) & 0x1;   /* validity_check와 동일값 */
    uint32_t payload  = (aud_sample >>  4) & 0x00FFFFFF;   /* bit[4-27] 24비트 */

    /* Non-audio 모드 갱신 — status_callback에서 CS byte[0]로 갱신되므로
       여기서는 validity=1인 경우만 즉시 반영 */
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
       IEC 61937 상태머신
       Pa(0xF872) -> Pb(0x4E1F) -> Pc(data-type) -> Pd(length)
       ft 타입으로 M/W 채널 순서 검증:
         Pa = sft_M (또는 sft_B)
         Pb = sft_W
         Pc = sft_M (또는 sft_B)
         Pd = sft_W
       → E-AC-3 데이터 내부에서 우연히 0xF872가 나와도
         ft 타입이 맞지 않으면 Pa로 인정하지 않음
    ------------------------------------------------------------------ */
    uint16_t word16 = (uint16_t)((aud_sample >> 12) & 0xFFFF);

    /* Pa/Pc 는 M 또는 B 채널, Pb/Pd 는 W 채널 */
    bool is_m_or_b = ( ft == sft_M || ft == sft_B );
    bool is_w      = ( ft == sft_W );

    switch ( mIecState )
    {
        case IEC61937_IDLE:
            /* Pa: 반드시 M(또는 B) 채널 서브프레임 */
            if ( 0xF872 == word16 && is_m_or_b ) {
                mIecState      = IEC61937_GOT_PA;
                mIecBurstStart = t;
                mIecPaFt       = ft;
            }
            break;

        case IEC61937_GOT_PA:
            /* Pb: 반드시 W 채널 + 값 0x4E1F */
            if ( 0x4E1F == word16 && is_w ) {
                mIecState = IEC61937_GOT_PB;
            } else if ( 0xF872 == word16 && is_m_or_b ) {
                /* Pa 재감지 → 시작점 갱신 */
                mIecBurstStart = t;
                mIecPaFt       = ft;
            } else {
                mIecState = IEC61937_IDLE;
            }
            break;

        case IEC61937_GOT_PB:
            /* Pc: 반드시 M(또는 B) 채널 서브프레임 */
            if ( is_m_or_b ) {
                mIecDataType = (uint8_t)(word16 & 0x1F);
                mIecState    = IEC61937_GOT_PC;
            } else {
                mIecState = IEC61937_IDLE;
            }
            break;

        case IEC61937_GOT_PC:
        {
            /* Pd: 반드시 W 채널 서브프레임 */
            if ( !is_w ) {
                mIecState = IEC61937_IDLE;
                break;
            }

            uint16_t payload_bits = word16;

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
       validity bit보다 더 정확한 Non-audio 판단 기준
       → gap 억제에 사용 */
    mIsNonAudio = ( status->channel_status_left[0] & 0x02 ) != 0;

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
        ( (uint64_t)status->channel_status_right[3] << 24 );

    csFrame.mFlags = 0;
    csFrame.mType  = FRAME_TYPE_CHANNEL_STATUS;
    csFrame.mStartingSampleInclusive = t;
    csFrame.mEndingSampleInclusive   = tend;
    mResults->AddFrame( csFrame );

    mPrevStatus    = t;
    mPrevStatusEnd = tend;
    mResults->CommitResults();   /* CS Frame을 UI에 반영 */
}
