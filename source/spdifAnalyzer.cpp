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
    mBSyncCount( 0 ),
    mIecEverDetected( false ),
    mStatusCallbackCount( 0 ),
    mLastCsData1( 0xFFFFFFFFFFFFFFFFULL ),
    mLastCsData2( 0xFFFFFFFFFFFFFFFFULL ),
    mWasNonAudio( false ),
    mLastIecType( 0xFF )
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
    mIecState         = IEC61937_IDLE;
    mIecDataType      = 0;
    mIecBurstStart    = 0;
    mIecPaFt          = sft_invalid;
    mIsNonAudio       = false;
    mBSyncCount          = 0;
    mIecEverDetected     = false;
    mStatusCallbackCount = 0;
    mLastCsData1         = 0xFFFFFFFFFFFFFFFFULL;
    mLastCsData2         = 0xFFFFFFFFFFFFFFFFULL;
    mWasNonAudio         = false;
    mLastIecType         = 0xFF;

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

        /* 신호 끊김 감지 → CS/IEC 캐시 리셋
           신호 재개 시 동일 값이어도 다시 표시 */
        mLastCsData1 = 0xFFFFFFFFFFFFFFFFULL;
        mLastCsData2 = 0xFFFFFFFFFFFFFFFFULL;
        mLastIecType = 0xFF;
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
        mBSyncCount++;   /* B-sync 카운터 증가 — Pa 허용 타이밍 판단용 */

        /* 상태머신이 중간에 있으면 리셋 (버스트 중간 B-sync는 무시) */
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
       ft 타입 검증: Pa/Pc = M or B,  Pb/Pd = W
       B-sync 이후 8 서브프레임 이내에서만 Pa 탐색
       → E-AC-3 데이터 내부 0xF872 오인식 차단
    ------------------------------------------------------------------ */
    uint16_t word16 = (uint16_t)((aud_sample >> 12) & 0xFFFF);

    bool is_m_or_b   = ( ft == sft_M || ft == sft_B );
    bool is_w        = ( ft == sft_W );

    /* Pa 허용: B-sync 윈도우 조건 제거 (v2와 동일)
       AC-3(8 B-sync), DTS(2~11 B-sync), E-AC-3(32 B-sync) 모두 커버
       오인식 차단은 Pc data-type 유효성 체크로만 */

    switch ( mIecState )
    {
        case IEC61937_IDLE:
            if ( 0xF872 == word16 && is_m_or_b ) {
                mIecState      = IEC61937_GOT_PA;
                mIecBurstStart = t;
                mIecPaFt       = ft;
            }
            break;

        case IEC61937_GOT_PA:
            if ( ft == sft_M ) {
                mIecState = IEC61937_GOT_PA_SKIP_M;
            } else if ( is_w ) {
                bool pb_ok = ( word16 == 0x4E1F || word16 == 0x4E3E );
                if ( pb_ok ) {
                    mIecState = IEC61937_GOT_PB;
                } else if ( 0xF872 == word16 ) {
                    mIecBurstStart = t; mIecPaFt = ft;
                } else {
                    mIecState = IEC61937_IDLE;
                }
            } else {
                mIecState = IEC61937_IDLE;
            }
            break;

        case IEC61937_GOT_PA_SKIP_M:
            if ( is_w ) {
                bool pb_ok = ( word16 == 0x4E1F || word16 == 0x4E3E );
                if ( pb_ok ) {
                    mIecState = IEC61937_GOT_PB;
                } else {
                    mIecState = IEC61937_IDLE;
                }
            } else {
                mIecState = IEC61937_IDLE;
            }
            break;

        case IEC61937_GOT_PB:
            if ( is_m_or_b ) {
                uint8_t dt = (uint8_t)(word16 & 0x1F);
                /* data-type 유효성 체크: 알려진 포맷만 허용
                   0x00 = null/미정의, 0x05/0x09~0x0A 등 미정의 타입 거부
                   오인식된 Pa→Pb 이후 엉뚱한 Pc 차단 */
                bool dt_valid = ( dt == 0x01 || dt == 0x02 || dt == 0x03 ||
                                  dt == 0x04 || dt == 0x06 || dt == 0x07 ||
                                  dt == 0x08 || dt == 0x0B || dt == 0x0C ||
                                  dt == 0x15 || dt == 0x16 );
                if ( dt_valid ) {
                    mIecDataType = dt;
                    mIecState    = IEC61937_GOT_PC;
                } else {
                    mIecState = IEC61937_IDLE;
                }
            } else {
                mIecState = IEC61937_IDLE;
            }
            break;

        case IEC61937_GOT_PC:
        {
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

            /* 직전과 동일한 포맷이면 표시 안 함 */
            if ( mIecDataType != mLastIecType ) {
                mLastIecType = mIecDataType;
                mResults->AddFrame( iecFrame );
                mResults->CommitResults();
                mResults->AddMarker( mIecBurstStart, AnalyzerResults::UpArrow,
                                     mSettings->mInputChannel );
            }

            mIecEverDetected = true;
            mBSyncCount      = 0;
            mIsNonAudio      = true;

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
    mStatusCallbackCount++;

    if ( mPrevStatus ) {
        mResults->CommitPacketAndStartNewPacket();
        mResults->CommitResults();
    }

    /* spdif.c에서 validity=1 서브프레임의 CS 비트를 수집하지 않으므로
       PCM 구간의 CS만 수집됨 → 쓰레기값 자동 제거
       mIsNonAudio 갱신: IEC61937 감지 후엔 PCM으로 와도 유지 */
    bool cs_is_nonaudio = ( status->channel_status_left[0] & 0x02 ) != 0;
    bool pcm_transition = mWasNonAudio && !cs_is_nonaudio;  /* Non-audio → PCM 전환 */

    if ( cs_is_nonaudio ) {
        mIsNonAudio  = true;
        mWasNonAudio = true;
    } else if ( !mIecEverDetected ) {
        mIsNonAudio  = false;
        mWasNonAudio = false;
    } else if ( pcm_transition ) {
        /* E-AC-3 → PCM 전환: mIsNonAudio 리셋, mIecEverDetected 리셋 */
        mIsNonAudio      = false;
        mWasNonAudio     = false;
        mIecEverDetected = false;
        mLastCsData1     = 0xFFFFFFFFFFFFFFFFULL;  /* 강제 표시 */
        mLastCsData2     = 0xFFFFFFFFFFFFFFFFULL;
    }

    memcpy( &mLastChannelStatus, status, sizeof(mLastChannelStatus) );
    mHasChannelStatus = true;

    mPrevStatus    = t;
    mPrevStatusEnd = tend;

    /* CS Frame 생성
       validity 필터(spdif.c)로 쓰레기값은 이미 제거됨 */

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
        ( mIsNonAudio ? ((uint64_t)1 << 32) : 0 );

    /* 직전 CS와 동일하면 표시 안 함 — 변화가 있을 때만 1회 표시 */
    if ( csFrame.mData1 == mLastCsData1 && csFrame.mData2 == mLastCsData2 ) {
        mResults->CommitResults();
        return;
    }

    mLastCsData1 = csFrame.mData1;
    mLastCsData2 = csFrame.mData2;

    /* PCM 전환 플래그: bit0=1이면 Non-audio→PCM 전환 시점 */
    csFrame.mFlags = pcm_transition ? 0x01 : 0x00;
    csFrame.mType  = FRAME_TYPE_CHANNEL_STATUS;
    csFrame.mStartingSampleInclusive = t;
    csFrame.mEndingSampleInclusive   = tend;
    mResults->AddFrame( csFrame );
    mResults->CommitResults();
}
