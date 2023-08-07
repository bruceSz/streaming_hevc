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
#pragma once

extern "C" {

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
//#include "libavutil/cpu.h"
//#include "libavutil/parseutils.h"
//#include <libswscale/swscale.h>
//#include <libavutil/imgutils.h>

}


namespace xpilot {
namespace perception {

class H265Encoder
{
public:
	static constexpr enum AVPixelFormat FORMAT = AV_PIX_FMT_YUV420P;
	static constexpr uint8_t ENDCODE[] = { 0, 0, 1, 0xb7 };
	H265Encoder() = default;
	~H265Encoder();

	virtual bool Init(int width, int height);
	virtual bool Encode(const AVFrame *frame, AVPacket* pkt);
	
private:

	void default_condec_context_config(int width, int height);
	AVFrame *frame = NULL;
	AVCodecContext *c_ = NULL;
	
};

class EncoderCBAdapter: public xpilot::perception::H265Encoder {
  using parr = xpilot::perception::H265Encoder;
  using Callback = std::function<void(const AVPacket* pkt)>;
  public:
    EncoderCBAdapter(Callback callback) {
      callback_ = std::move(callback);
    }
    bool Init(int width, int height) override {
      XINFO << "CB adapter init called." << std::endl;
      parr::Init(width, height);
      XINFO << "CB adapter init called done." << std::endl;
      return true;
    }

    bool Encode(const AVFrame *frame, AVPacket* pkt) override {
      XINFO << "Encoder writer adapter Encode called." << std::endl;
      auto ret = parr::Encode(frame, pkt);
      XCHECK(ret);
      if (pkt->size > 0) {
        XINFO << i << " frame to be write" << std::endl;
        callback_(pkt);
      } else {
        XERROR << "encode return empty size for frame: " << i++ << std::endl;
      }
      return true;

    }

    void finilize() {
      int left_over_cnt = i;
      AVPacket pkt;
      while (true) {
          auto ret = parr::Encode(nullptr, &pkt);
          XCHECK(ret);
          if (pkt.size > 0) {
              XINFO << "left over write frame: " << left_over_cnt << std::endl;
              callback_(&pkt);
              av_free_packet(&pkt);
          } else {
            break;
          }
          left_over_cnt++;
      }
      // call this before 
     // fwrite(xpilot::perception::H265Encoder::ENDCODE, 1, sizeof(xpilot::perception::H265Encoder::ENDCODE), fh_);
	 return;
    }

    ~EncoderCBAdapter() {
      finilize();
    }

	void RegisterCallback(const Callback &callback) {
		callback_ = callback;
	}

  private:
  int i;
   //xpilot::perception::H265Encoder encoder;
   Callback callback_;

   
};


}	
}
