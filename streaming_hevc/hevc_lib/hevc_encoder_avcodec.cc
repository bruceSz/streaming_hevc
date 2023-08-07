/* Copyright (c) 2023-2030, XPeng Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *     * Neither the name of XPeng Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * ---
 * Author: Bruce S.Zhang
 */

#include "infra/common/log.h"

#include "hevc_flags.h"
#include "hevc_encoder_x86.h"

extern "C" {
#include <libavutil/imgutils.h>
}

namespace xpilot {
namespace perception {



H265Encoder::~H265Encoder() {
	avcodec_free_context(&c_);
}

void H265Encoder::default_condec_context_config(int width, int height) {
    /* put sample parameters */
    c_->bit_rate = 400000;
    /* resolution must be a multiple of two */
    c_->width = width;
    c_->height = height;
    /* frames per sec_ond */
    c_->time_base = (AVRational){1,25};

     /* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */
    c_->gop_size = 10;
    c_->max_b_frames = 1;
    c_->pix_fmt = H265Encoder::FORMAT;


    /*
	c->codec_type = AVMEDIA_TYPE_VIDEO;
	c->pix_fmt = AV_PIX_FMT_YUV420P;
	c->width = width;
	c->height = height;
	c->time_base.num = 1;
	c->time_base.den = 25;
	c->bit_rate = 900000;

	c->qmin = 10;
	c->qmax = 30;
    */

   return;
}

bool H265Encoder::Init(int width, int height) {
	 AVCodec *codec;

    /* find and alloc context encoder */
    codec = avcodec_find_encoder(static_cast<enum AVCodecID>(AV_CODEC_ID_HEVC));
    XCHECK_NOTNULL(codec);
    
    c_ = avcodec_alloc_context3(codec);
    XCHECK_NOTNULL(c_);
    XINFO << "avcodec context: " << c_ << std::endl;

    //TODO, check affect, after remove this.    
    default_condec_context_config(width, height);

    //if (codec_id == AV_CODEC_ID_H264)
    //    av_opt_set(c->priv_data, "preset", "slow", 0);

    /* open it */
    if (avcodec_open2(c_, codec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }
	return true;
}

bool H265Encoder::Encode(const AVFrame *frame, AVPacket* pkt) {

    int i, ret,zx,x,y, got_output;
    

    av_init_packet(pkt);
    pkt->data = NULL;    // packet data will be allocated by the encoder
    pkt->size = 0;
    
     /* encode the image */
    ret = avcodec_encode_video2(c_, pkt, frame, &got_output);
    if (ret < 0) {
       XERROR << "Encode frame failed." << std::endl;
        return false;
    }

    if (got_output) {
        XINFO << "got encoded pkt frokm frame "  <<" (size=" << pkt->size << ")";
//        fwrite(pkt.data, 1, pkt.size, f);
        
    }
    return true;
    /* get the delayed frames */
    

	/*if (frame)
	veLog::Info("H265Encoder:Send frame %3lld\n", frame->pts);*/

    /*
	int ret = avcodec_send_frame(c, frame);
	if (ret < 0)
	{
		printf("H265Encoder:Error sending a frame for encoding!\n");
		return;
	}
	while (ret >= 0)
	{
		ret = avcodec_receive_packet(c, packet);
		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
			return;
		}
		else if (ret < 0)
		{
			printf("H265Encoder:Error during encoding!\n");
			return;
		}

		memcpy(m_h264, packet->data, packet->size);
		pOutData = m_h264;
		iSize = packet->size;
		av_packet_unref(packet);
	}
    */
   
}

}
}