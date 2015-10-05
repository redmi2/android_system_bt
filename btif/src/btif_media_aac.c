/******************************************************************************
 *
 *  Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *   met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include "aacdecoder_lib.h"
#include "btif_media.h"
#include "a2d_aac.h"
#ifdef USE_AUDIO_TRACK
#include "bluetoothTrack.h"
#endif

static HANDLE_AACDECODER decoder = NULL;
static CStreamInfo *streamInfo = NULL;

#ifdef PCM_DUMP
#include "btif_a2dp_pcm_dump.h"
#endif
#define OUTPUT_SIZE (8*2*1024)

/* MAX Layers are made 3  because of KW error.
 * Though we set number of layers as 1 while opening decoder
 * Other indexes will not be used
 */
#define MAX_NUM_LAYERS 3

/* decode_buf captures actual decoded data */
/* output_buf is created from decode buf, it will have */
/* right and left channel data */

INT16 decode_buf[OUTPUT_SIZE/2];
UINT8 output_buf[OUTPUT_SIZE];

/* Object type: MPEG-2 AAC LC (Currently supported type)
 * Has to be transcoded to MPEG-4 LATM at source
 * AudioMuxEliment Inband mode
 */

/* AAC format: ADTS (Just for reference now)
 * 7 Byte header without CRC, 9 Byte with CRC
 * SSSSSSSS SSSSVYYP RRFFFFUC CCUUUULL LLLLLLLL LLLBBBBB BBBBBBNN
 * S - Sync word
 * V - MPEG version
 * Y - Layer
 * P - Protection
 * R - Profile
 * F - Sampling Frequency
 * U - Unused
 * C - Channel mode
 * L - Frame Length
 * B - Buffer Fullness
 * N - Number of AAC frames in ADTS frame
 */
int btif_a2dp_get_aac_track_frequency(UINT16 frequency) {
    int freq = 44100;
    switch (frequency) {
        case A2D_AAC_IE_SAMP_FREQ_16000:
            freq = 16000;
            break;
        case A2D_AAC_IE_SAMP_FREQ_32000:
            freq = 32000;
            break;
        case A2D_AAC_IE_SAMP_FREQ_44100:
            freq = 44100;
            break;
        case A2D_AAC_IE_SAMP_FREQ_48000:
            freq = 48000;
            break;
    }
    return freq;
}

int btif_a2dp_get_aac_track_channel_count(UINT8 channeltype) {
    int count = 2;
    switch (channeltype) {
        case A2D_AAC_IE_CHANNELS_1:
            count = 1;
            break;
        case A2D_AAC_IE_CHANNELS_2:
/* If we use AudioTrack for rendering data, we need to set */
/* channel value as 3, for stereo */
#ifdef USE_AUDIO_TRACK
            count = 3;
#else
            count = 2;
#endif
            break;
    }
    return count;
}
/*******************************************************************************
 **
 ** Function         btif_media_acc_close_decoder
 **
 ** Description      Close AAC Decoder.
 **
 ** Returns          void
 **
 *******************************************************************************/
void btif_media_acc_close_decoder()
{
    if (decoder != NULL)
    {
        APPL_TRACE_ERROR("btif_media_acc_close_decoder");
        aacDecoder_Close(decoder);
        decoder = NULL;
    }
}
/*******************************************************************************
 **
 ** Function         btif_media_aac_decoder_reset
 **
 ** Description      AAC decoder initialization.
 **                  Currently only MPEG_2_AAC_LC is supported, for this
 **                  stream has to be encoded to MPEG-4 LATM.
 ** Returns          void
 **
 *******************************************************************************/
void btif_media_aac_decoder_reset(BT_HDR *p_msg,UINT16 *sampling, UINT8 *channel_mode)
{
    tBTIF_MEDIA_SINK_CFG_UPDATE *p_buf = (tBTIF_MEDIA_SINK_CFG_UPDATE*) p_msg;
    tA2D_STATUS a2d_status;
    tA2D_AAC_CIE aac_cie;
    TRANSPORT_TYPE transport;

    APPL_TRACE_DEBUG("aac_decoder_reset codec_info[%x:%x:%x:%x:%x:%x]",
            p_buf->codec_info[1], p_buf->codec_info[2], p_buf->codec_info[3],
            p_buf->codec_info[4], p_buf->codec_info[5], p_buf->codec_info[6]);

    a2d_status = A2D_ParsAacInfo (&aac_cie, p_buf->codec_info, FALSE);
    if (a2d_status == A2D_SUCCESS)
    {
        *sampling = btif_a2dp_get_aac_track_frequency(aac_cie.samp_freq);
        *channel_mode = btif_a2dp_get_aac_track_channel_count(aac_cie.channels);
        APPL_TRACE_DEBUG(" samp_freq = %d, channels = %d", *sampling, *channel_mode);
    }
    else
    {
        APPL_TRACE_ERROR("aac_decoder_reset: Failed parsing codec info");
        return;
    }
    /* Currenty supporting only MPEG-2 AAC LC
     */
    switch(aac_cie.object_type)
    {
        case A2D_AAC_IE_OBJ_TYPE_MPEG_2_AAC_LC:
            /* Need to check if we have to depend on Marker bit in RTP header */
            transport = TT_MP4_LATM_MCP1;
            break;
        default:
            APPL_TRACE_ERROR("aac_decoder_reset: Unsupported format");
            return;
    }

    /* AAC decoder initialization */

    if (decoder != NULL)
    {
        APPL_TRACE_ERROR("aac_decoder_reset: Decoder instance already running");
        aacDecoder_Close(decoder);
#ifdef PCM_DUMP
        openDumpFile();
#endif
    }
    decoder = aacDecoder_Open(transport, 1);
    if(decoder == NULL)
    {
        APPL_TRACE_ERROR("aac_decoder_reset: Error opening decoder instance");
        return;
    }
#ifdef USE_AUDIO_TRACK
    APPL_TRACE_DEBUG("A2dpSink: aac Create Track");
    if (btCreateTrack(btif_a2dp_get_aac_track_frequency(aac_cie.samp_freq), btif_a2dp_get_aac_track_channel_count(aac_cie.channels)) == -1) {
        APPL_TRACE_ERROR("A2dpSink: Track creation fails!!!");
        return;
    }
#endif
    return;
}

/*******************************************************************************
 **
 ** Function         btif_media_aac_decode
 **
 ** Description      AAC decode
 **                  Takes the media payload as the input and len in BT_HDR is
 **                  updated after decode to indicate the bytes left after decoder
 **                  fill. Decoded PCM is written to UIPC or wav file based on
 **                  the configuration(WAV_DUMP definition)
 ** Returns          void
 **
 *******************************************************************************/
void btif_media_aac_decode(BT_HDR *p_msg)
{
    UINT8 *ptr[MAX_NUM_LAYERS];
    UINT packet_size[MAX_NUM_LAYERS], frame_size, index;
    UINT valid;
    AAC_DECODER_ERROR err;
#ifdef USE_AUDIO_TRACK
    int retwriteAudioTrack = 0;
#endif
    if (decoder == NULL)
    {
        APPL_TRACE_ERROR("aac_decode: Decoder instance not ready");
        return;
    }
    ptr[0] = (UINT8*)(p_msg + 1) + p_msg->offset;
    packet_size[0] = p_msg->len;
    valid = packet_size[0];
    APPL_TRACE_DEBUG("btif_media_aac_decode pkt_size %d ",packet_size[0]);
    err = aacDecoder_Fill(decoder, ptr, packet_size, &valid);
    if (err != AAC_DEC_OK)
    {
        APPL_TRACE_ERROR("aac_decode: Error in FillBuffer");
        return;
    }
    if (valid == 0)
    {
        p_msg->len = 0;
    }
    else
    {
        p_msg->len = valid;
        p_msg->offset += (packet_size[0] - valid);
    }
    err = aacDecoder_DecodeFrame(decoder, decode_buf, OUTPUT_SIZE, 0);
    if(err == AAC_DEC_NOT_ENOUGH_BITS)
    {
        APPL_TRACE_ERROR("aac_decode: Decode not enough bits continue");
        return;
    }
    if (err != AAC_DEC_OK)
    {
        APPL_TRACE_ERROR("aac_decode: Error in Decode");
        return;
    }
    streamInfo = aacDecoder_GetStreamInfo(decoder);

    frame_size = streamInfo->frameSize;
    for(index = 0; index < frame_size * streamInfo->numChannels; index++)
    {
        UINT8 *out = &output_buf[2*index];

        out[0] = decode_buf[index] & 0xff;
        out[1] = decode_buf[index] >> 8;
    }
#ifdef PCM_DUMP
    writeDumpFile((void*)output_buf, (streamInfo->numChannels * PCM_SAMPLE_SIZE * frame_size));
#endif
#ifdef USE_AUDIO_TRACK
    retwriteAudioTrack = btWriteData((void*)output_buf, (streamInfo->numChannels * PCM_SAMPLE_SIZE * frame_size));
#else
    UIPC_Send(UIPC_CH_ID_AV_AUDIO, 0, (UINT8 *)output_buf,
            streamInfo->numChannels * PCM_SAMPLE_SIZE * frame_size);
#endif
}

