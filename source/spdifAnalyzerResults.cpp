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
#include "spdifAnalyzerResults.h"
#include <AnalyzerHelpers.h>
#include "spdifAnalyzer.h"
#include "spdifAnalyzerSettings.h"
#include <iostream>
#include <fstream>
#include <string.h>

/* ======================================================================
   IEC 61937 data-type 코드를 문자열로 변환
   ====================================================================== */
static const char* iec61937_type_name( uint8_t dt )
{
    switch ( dt & 0x1F )
    {
        case IEC61937_AC3:       return "AC-3 (Dolby Digital)";
        case IEC61937_MPEG1_L1:  return "MPEG-1 Layer1";
        case IEC61937_MPEG1_L23: return "MPEG-1 Layer2/3";
        case IEC61937_MPEG2_LSF: return "MPEG-2 LSF";
        case IEC61937_DTS_I:     return "DTS type-I";
        case IEC61937_DTS_II:    return "DTS type-II";
        case IEC61937_DTS_III:   return "DTS type-III";
        case IEC61937_DTS_IV:    return "DTS type-IV (DTS-HD MA)";
        case IEC61937_MPEG2_AAC: return "MPEG-2 AAC";
        case IEC61937_EAC3:      return "E-AC-3 (Dolby Digital Plus)";
        case IEC61937_TRUEHD:    return "Dolby TrueHD / MLP";
        default:                 return "Unknown Non-audio";
    }
}

/* ======================================================================
   Channel Status byte[3] 하위 4비트 → 샘플링 주파수 문자열
   ====================================================================== */
static const char* cs_sample_rate( uint8_t cs3, bool is_nonpcm = false )
{
    switch ( cs3 & 0x0F )
    {
        /* Non-audio(IEC61937)에서 byte[3]=0x00은 192kHz 버스트를 의미 */
        case 0x00: return is_nonpcm ? "192kHz(IEC61937)" : "44.1kHz";
        case 0x02: return "48kHz";
        case 0x03: return "32kHz";
        case 0x08: return "88.2kHz";
        case 0x0A: return "96kHz";
        case 0x0C: return "176.4kHz";
        /* 0x0E: PCM이면 192kHz, Non-audio이면 192kHz(IEC61937)로 통일 */
        case 0x0E: return is_nonpcm ? "192kHz(IEC61937)" : "192kHz";
        default:   return "?kHz";
    }
}

/* ======================================================================
   Channel Status byte[4] 하위 4비트 → Word length 문자열
   IEC 60958-3 Table 7
   ====================================================================== */
static const char* cs_word_length( uint8_t cs4 )
{
    /* bit[0]: max word length  0=20bit max, 1=24bit max
       bit[3:1]: word length setting */
    bool max24 = (cs4 & 0x01) != 0;
    switch ( (cs4 >> 1) & 0x07 )
    {
        case 0x00: return max24 ? "24bit(notindicated)" : "20bit(notindicated)";
        case 0x01: return max24 ? "24bit" : "20bit";
        case 0x02: return max24 ? "22bit" : "18bit";
        case 0x04: return max24 ? "23bit" : "19bit";
        case 0x05: return max24 ? "16bit" : "16bit";
        case 0x06: return max24 ? "21bit" : "17bit";
        default:   return "?bit";
    }
}

/* ====================================================================== */

spdifAnalyzerResults::spdifAnalyzerResults( spdifAnalyzer* analyzer, spdifAnalyzerSettings* settings )
:   AnalyzerResults(),
    mSettings( settings ),
    mAnalyzer( analyzer )
{
}

spdifAnalyzerResults::~spdifAnalyzerResults()
{
}

/* ======================================================================
   GenerateBubbleText — 파형 위 말풍선 텍스트
   ====================================================================== */
void spdifAnalyzerResults::GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base )
{
    ClearResultStrings();
    Frame frame = GetFrame( frame_index );
    char buf[128];

    /* ---- Channel Status 블록 ---- */
    if ( (uint8_t)frame.mType == FRAME_TYPE_CHANNEL_STATUS )
    {
        uint8_t cs0 = (uint8_t)((frame.mData1 >>  0) & 0xFF);
        uint8_t cs3 = (uint8_t)((frame.mData1 >> 24) & 0xFF);
        uint8_t cs4 = (uint8_t)((frame.mData1 >> 32) & 0xFF);

        const char *audio_type = (cs0 & 0x02) ? "Non-audio" : "PCM";
        bool        is_nonpcm  = (cs0 & 0x02) != 0;
        const char *fs         = cs_sample_rate( cs3, is_nonpcm );
        const char *wl         = cs_word_length( cs4 );

        snprintf( buf, sizeof(buf), "CS:%s %s %s", audio_type, fs, wl );
        AddResultString( buf );
        return;
    }

    /* ---- IEC 61937 포맷 감지 ---- */
    if ( (uint8_t)frame.mType == FRAME_TYPE_IEC61937 )
    {
        uint8_t dt = (uint8_t)(frame.mData1 & 0xFF);
        snprintf( buf, sizeof(buf), "IEC61937:%s", iec61937_type_name(dt) );
        AddResultString( buf );
        return;
    }

    /* ---- 일반 오디오 / 에러 프레임 (기존 로직) ---- */
    if ( sft_invalid == frame.mType )
        return;

    if ( (Decimal == display_base) || (ASCII == display_base) ) {
        snprintf( buf, sizeof(buf), "%d", (int)frame.mData1 );
    } else {
        AnalyzerHelpers::GetNumberString( frame.mData1 & 0x0000ffff, display_base, 16, buf, 128 );
    }
    AddResultString( buf );
}

/* ======================================================================
   GenerateExportFile — CSV / WAV / RAW 내보내기 (기존 로직 유지)
   ====================================================================== */
void spdifAnalyzerResults::GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id )
{
    if ( 0 == export_type_user_id ) /* text/csv */
    {
        std::ofstream file_stream( file, std::ios::out );

        U64 trigger_sample = mAnalyzer->GetTriggerSample();
        U32 sample_rate    = mAnalyzer->GetSampleRate();

        file_stream << "Time [s],Type,Value" << std::endl;

        U64 num_frames = GetNumFrames();
        for ( U32 i = 0; i < num_frames; i++ )
        {
            Frame frame = GetFrame( i );

            char time_str[128];
            AnalyzerHelpers::GetTimeString( frame.mStartingSampleInclusive, trigger_sample, sample_rate, time_str, 128 );

            if ( (uint8_t)frame.mType == FRAME_TYPE_CHANNEL_STATUS )
            {
                uint8_t cs0 = (uint8_t)((frame.mData1 >>  0) & 0xFF);
                uint8_t cs1 = (uint8_t)((frame.mData1 >>  8) & 0xFF);
                uint8_t cs2 = (uint8_t)((frame.mData1 >> 16) & 0xFF);
                uint8_t cs3 = (uint8_t)((frame.mData1 >> 24) & 0xFF);
                uint8_t cs4 = (uint8_t)((frame.mData1 >> 32) & 0xFF);

                const char *audio_type = (cs0 & 0x02) ? "Non-audio" : "PCM";
                const char *copy       = (cs0 & 0x04) ? "CopyOK"    : "NoCopy";
                const char *emph       = (cs0 & 0x08) ? "Emphasis"  : "NoEmph";
                bool        is_nonpcm  = (cs0 & 0x02) != 0;

                char row[512];
                snprintf( row, sizeof(row),
                    "%s,ChStatus,%s|%s|%s|%s|Cat:0x%02X|Src:%d|Ch:%d|Wlen:%s",
                    time_str,
                    audio_type, copy, emph,
                    cs_sample_rate(cs3, is_nonpcm),
                    cs1,
                    (cs2 >> 0) & 0x0F,
                    (cs2 >> 4) & 0x0F,
                    cs_word_length(cs4) );
                file_stream << row << std::endl;
            }
            else if ( (uint8_t)frame.mType == FRAME_TYPE_IEC61937 )
            {
                uint8_t  dt           = (uint8_t)(frame.mData1 & 0xFF);
                uint16_t payload_bits = (uint16_t)(frame.mData2 & 0xFFFF);
                char row[256];
                snprintf( row, sizeof(row),
                    "%s,IEC61937,%s|PayloadBits:%u",
                    time_str, iec61937_type_name(dt), payload_bits );
                file_stream << row << std::endl;
            }
            else
            {
                char number_str[128];
                AnalyzerHelpers::GetNumberString( frame.mData1, display_base, 8, number_str, 128 );
                file_stream << time_str << ",PCM," << number_str << std::endl;
            }

            if ( UpdateExportProgressAndCheckForCancel( i, num_frames ) )
                break;
        }
        file_stream.close();
    }
    else if ( 1 == export_type_user_id ) /* wav */
    {
        std::ofstream file_stream( file, std::ios::binary );

        struct WAVHeader wh;
        U64 num_frames  = GetNumFrames();
        U64 num_samples = 0;

        wh_Init( &wh, (uint32_t)num_frames >> 1 );
        file_stream.write( (const char *)&wh, sizeof(wh) );

        for ( U32 i = 0; i < num_frames; i++ )
        {
            Frame frame = GetFrame( i );

            /* Channel Status / IEC61937 프레임은 WAV에 포함하지 않음 */
            uint8_t ftype = (uint8_t)frame.mType;
            if ( ftype == FRAME_TYPE_CHANNEL_STATUS || ftype == FRAME_TYPE_IEC61937 )
                continue;

            if ( !(0x80 & frame.mType) )
            {
                uint16_t pcm = (uint16_t)frame.mData1;
                file_stream.write( (const char *)&pcm, sizeof(pcm) );
                num_samples++;
            }

            if ( UpdateExportProgressAndCheckForCancel( i, num_frames ) )
                break;
        }

        wh_Init( &wh, (uint32_t)num_samples >> 1 );
        file_stream.seekp( 0 );
        file_stream.write( (const char *)&wh, sizeof(wh) );
        file_stream.close();
    }
    else if ( 2 == export_type_user_id ) /* raw/bin */
    {
        std::ofstream file_stream( file, std::ios::binary );

        U64 num_frames = GetNumFrames();
        for ( U32 i = 0; i < num_frames; i++ )
        {
            Frame frame = GetFrame( i );

            uint8_t ftype = (uint8_t)frame.mType;
            if ( ftype == FRAME_TYPE_CHANNEL_STATUS || ftype == FRAME_TYPE_IEC61937 )
                continue;

            if ( !(0x80 & frame.mType) )
            {
                uint32_t raw = (uint32_t)frame.mData2;
                file_stream.write( (const char *)&raw, sizeof(raw) );
            }

            if ( UpdateExportProgressAndCheckForCancel( i, num_frames ) )
                break;
        }
        file_stream.close();
    }
}

/* ======================================================================
   GenerateFrameTabularText — Saleae 데이터 테이블 행 텍스트
   ====================================================================== */
void spdifAnalyzerResults::GenerateFrameTabularText( U64 frame_index, DisplayBase display_base )
{
    Frame frame = GetFrame( frame_index );
    ClearTabularText();

    /* ---- Channel Status 블록 ---------------------------------------- */
    if ( (uint8_t)frame.mType == FRAME_TYPE_CHANNEL_STATUS )
    {
        uint8_t cs0 = (uint8_t)((frame.mData1 >>  0) & 0xFF);
        uint8_t cs1 = (uint8_t)((frame.mData1 >>  8) & 0xFF);
        uint8_t cs2 = (uint8_t)((frame.mData1 >> 16) & 0xFF);
        uint8_t cs3 = (uint8_t)((frame.mData1 >> 24) & 0xFF);
        uint8_t cs4 = (uint8_t)((frame.mData1 >> 32) & 0xFF);

        uint8_t rcs0 = (uint8_t)((frame.mData2 >>  0) & 0xFF);
        bool force_nonpcm = ( (frame.mData2 >> 32) & 0x1 ) != 0;
        bool pcm_transition = ( frame.mFlags & 0x01 ) != 0;

        bool is_nonpcm = ( cs0 & 0x02 ) != 0 || force_nonpcm;
        const char *audio_type = is_nonpcm ? "Non-audio" : "PCM";
        const char *copy       = (cs0 & 0x04) ? "CopyOK"    : "NoCopy";
        const char *emph       = (cs0 & 0x08) ? "Emphasis"  : "NoEmph";
        const char *prof       = ( (cs0 & 0x01) && !force_nonpcm ) ? "Pro" : "Consumer";

        uint32_t rcs_raw = (uint32_t)(frame.mData2 & 0xFFFFFFFF);
        bool right_cs_valid = ( rcs_raw != 0 );
        const char *lr_match = ( !right_cs_valid || cs0 == rcs0 ) ? "" : " [L/R mismatch!]";

        char buf[512];

        /* Non-audio → PCM 전환 시 [IEC60958] PCM 먼저 표시 */
        if ( pcm_transition ) {
            snprintf( buf, sizeof(buf), "[IEC60958] PCM\n" );
            AddTabularText( buf );
        }

        snprintf( buf, sizeof(buf),
            "[ChStatus] %s | %s | %s | %s | %s | Cat:0x%02X | Src:%d | Ch:%d | Wlen:%s%s\n",
            prof, audio_type, copy, emph,
            cs_sample_rate( cs3, is_nonpcm ),
            cs1,
            (cs2 >> 0) & 0x0F,
            (cs2 >> 4) & 0x0F,
            cs_word_length( cs4 ),
            lr_match );
        AddTabularText( buf );
        return;
    }

    /* ---- IEC 61937 포맷 감지 프레임 ---------------------------------- */
    if ( (uint8_t)frame.mType == FRAME_TYPE_IEC61937 )
    {
        uint8_t  dt           = (uint8_t)(frame.mData1 & 0xFF);
        uint16_t payload_bits = (uint16_t)(frame.mData2 & 0xFFFF);
        char buf[256];
        snprintf( buf, sizeof(buf),
            "[IEC61937] %s | PayloadBits:%u\n",
            iec61937_type_name(dt), payload_bits );
        AddTabularText( buf );
        return;
    }

    /* ---- 일반 오디오 프레임 (기존 로직) ------------------------------- */
    char num1_str[128];

    switch ( frame.mType )
    {
        case sft_B:
        case (sft_B | 0x80):
        case sft_M:
        case sft_W:
            return;

        case sft_invalid:
        default:
            AnalyzerHelpers::GetNumberString( frame.mData1 & 0x0000ffff, display_base, 16, num1_str, 128 );
            AddTabularText( "T:err", " samp:", num1_str );
            break;
    }
}

void spdifAnalyzerResults::GeneratePacketTabularText( U64 packet_id, DisplayBase display_base )
{
    ClearResultStrings();
    AddResultString( "not supported" );
}

void spdifAnalyzerResults::GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base )
{
    ClearResultStrings();
    AddResultString( "not supported" );
}
