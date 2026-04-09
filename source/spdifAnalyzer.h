#ifndef SPDIF_ANALYZER_H
#define SPDIF_ANALYZER_H
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

#include <Analyzer.h>
#include "spdifAnalyzerResults.h"
#include "spdifSimulationDataGenerator.h"

extern "C" {
#include <stdint.h>
#include "spdif.h"
#include "wavhdr.h"
};

/* ------------------------------------------------------------------ */
/* 플러그인 버전                                                         */
#define SPDIF_ANALYZER_VERSION  "v17"

/* ------------------------------------------------------------------ */
/* IEC 61937 버스트 구조 파싱용 상태머신                                 */
/*  Pa(0xF872) -> Pb(0x4E1F) -> Pc(data-type) -> Pd(length)           */
/* ------------------------------------------------------------------ */
enum Iec61937State {
    IEC61937_IDLE,          /* Pa 대기 중                    */
    IEC61937_GOT_PA,        /* Pa 감지, M 스킵 대기          */
    IEC61937_GOT_PA_SKIP_M, /* M 1개 스킵 완료, Pb(W) 대기   */
    IEC61937_GOT_PB,        /* Pb 감지, Pc 대기              */
    IEC61937_GOT_PC,        /* Pc 감지, Pd 대기              */
};

/* IEC 61937 Pc 워드 하위 5비트 data-type 코드 */
#define IEC61937_AC3        0x01
#define IEC61937_MPEG1_L1   0x02
#define IEC61937_MPEG1_L23  0x03
#define IEC61937_MPEG2_LSF  0x04
#define IEC61937_DTS_I      0x06
#define IEC61937_DTS_II     0x07
#define IEC61937_DTS_III    0x08
#define IEC61937_DTS_IV     0x0B
#define IEC61937_MPEG2_AAC  0x0C
#define IEC61937_EAC3       0x15
#define IEC61937_TRUEHD     0x16

/* Frame mType 구분 상수 */
#define FRAME_TYPE_CHANNEL_STATUS   0xFE   /* Channel Status 블록 프레임 */
#define FRAME_TYPE_IEC61937         0xFD   /* IEC 61937 포맷 감지 프레임  */
#define FRAME_TYPE_DBG_PA           0xFC   /* 디버그: Pa 감지             */
#define FRAME_TYPE_DBG_PB           0xFB   /* 디버그: Pb 감지/실패        */
#define FRAME_TYPE_DBG_PC           0xFA   /* 디버그: Pc 감지/실패        */
#define FRAME_TYPE_DBG_PD           0xF9   /* 디버그: Pd 감지/실패        */

class spdifAnalyzerSettings;
class ANALYZER_EXPORT spdifAnalyzer : public Analyzer2
{
public:
    spdifAnalyzer();
    virtual ~spdifAnalyzer();
    virtual void WorkerThread();

    virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
    virtual U32 GetMinimumSampleRateHz();

    virtual const char* GetAnalyzerName() const;
    virtual bool NeedsRerun();
    virtual void SetupResults();

    /* callbacks from the "C" bitstream analyzer library */
    void sample_callback( uint64_t t, uint64_t tend, enum SpdifFrameType ft, uint32_t aud_sample );
    void status_callback( uint64_t t, uint64_t tend, struct SpdifChannelStatus *status );

protected: //vars
    std::auto_ptr< spdifAnalyzerSettings > mSettings;
    std::auto_ptr< spdifAnalyzerResults >  mResults;
    AnalyzerChannelData*                   mSerial;

    spdifSimulationDataGenerator mSimulationDataGenerator;
    bool                         mSimulationInitilized;

    /* Serial analysis vars */
    U32 mSampleRateHz;
    U32 mStartOfStopBitOffset;
    U32 mEndOfStopBitOffset;

    U16 m_PrevPCM;
    U16 m_Pad0;
    U32 m_AC3_Detected;

    /* "C" bitstream parser library */
    struct SpdifBitstreamAnalyzer *mSba;
    uint64_t                       mSamplesSinceLastBSync;
    uint64_t                       mPrevSample;
    uint64_t                       mPrevSampleEnd;
    uint64_t                       mPrevStatus;
    uint64_t                       mPrevStatusEnd;

    /* ---- Channel Status (IEC 60958) -------------------------------- */
    struct SpdifChannelStatus      mLastChannelStatus;
    bool                           mHasChannelStatus;

    /* ---- IEC 61937 Non-audio 포맷 감지 상태머신 -------------------- */
    enum Iec61937State      mIecState;      /* 현재 상태                    */
    uint8_t                 mIecDataType;   /* Pc 워드에서 읽은 data-type   */
    uint64_t                mIecBurstStart; /* Pa 감지 시각                 */
    enum SpdifFrameType     mIecPaFt;       /* Pa의 프리앰블 타입 (M/W)     */
    bool                    mIsNonAudio;    /* Non-audio 모드 여부          */

    /* ---- B-sync 카운터 ---------------------------------------------- */
    /* E-AC-3 버스트 = 6144 서브프레임 = 32 × 192(B-sync 간격)
       진짜 Pa가 있는 B-sync는 32번 중 딱 1번
       IEC61937 감지 성공 후 B-sync를 세어 32번째에서만 Pa 허용
       → 나머지 31번 B-sync의 sft_B 오인식 완전 차단              */
    uint32_t                mBSyncCount;    /* 마지막 IEC61937 감지 후 B-sync 횟수 */
    bool                    mIecEverDetected; /* 한 번이라도 IEC61937 감지됐는지  */
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //SPDIF_ANALYZER_H
