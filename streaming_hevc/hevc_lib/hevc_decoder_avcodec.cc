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

#include <iostream>
#include <chrono>
#include <memory>


#include "libde265/pps.h"
#include "libde265/image.h"
#include "infra/common/log.h"
#include "hevc_flags.h"
#include "opencv2/opencv.hpp"

#include "infra/common/log.h"
#include "perception/common/utils/general_utils.hpp"

#include "hevc_decoder_x86.h"
#include "hevc_test_common.h"

namespace xpilot {
namespace perception {


HevcDecoderAvcodec::~HevcDecoderAvcodec() {
  avcodec_close(codec_context_);
  av_free(codec_context_);
  av_frame_free(&frame);

}

void HevcDecoderAvcodec::Init() {
   codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    XCHECK_NOTNULL(codec);
    codec_context_ = avcodec_alloc_context3(codec);
    XCHECK_NOTNULL(codec_context_);
    codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_context_->thread_count = 1;

     /* open it */
    XCHECK_GE (avcodec_open2(codec_context_, codec, NULL) , 0);

    frame = av_frame_alloc();
    XCHECK_NOTNULL(frame);

    HevcDecoder::Init();
    XINFO << "HevcDecoderAvcodec initialized" << std::endl;
    
    return ;
}

bool HevcDecoderAvcodec::sendBlock(const char* block, uint64_t len) {
  // allocate new fmt ctx.
  fmt_ctx_ = avformat_alloc_context();
  
  io_ctx_ = avio_alloc_context(reinterpret_cast<uchar*>(const_cast<char*>(block)), len,
                                    0, NULL, NULL, NULL, NULL);
  io_ctx_->seekable = 0;
  //AVInputFormat *fmt_in;
  fmt_ctx_->pb = io_ctx_;

  int ret = avformat_open_input(&fmt_ctx_, "tmp name", NULL, NULL);
  XCHECK_GE(ret, 0);
  

}

bool HevcDecoderAvcodec::getNextYUV(unsigned char**, int*, int*, int*) {

}

static
int decode_packet(AVPacket *pPacket, AVCodecContext *pCodecContext, AVFrame *pFrame, bool scale) {
  static int frame_count = 0;
  static int last_ts = -1;
    auto b = now_in_nanoseconds();
  // Supply raw packet data as input to a decoder
  // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
  int response = avcodec_send_packet(pCodecContext, pPacket);

  if (response < 0) {
    
     XERROR << "Error while sending a packet to the decoder: " << response;
    return response;
  }

  while (response >= 0)
  {
    // Return decoded output data (into a frame) from a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga11e6542c4e66d3028668788a1a74217c
    response = avcodec_receive_frame(pCodecContext, pFrame);
    if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
      XINFO << "Need more read." << std::endl;
      break;
    } else if (response < 0) {

       XINFO << "Error while receiving a frame from the decoder: " << response ;
      return response;
    }

    if (response >= 0) {
      fprintf(stdout,
          "Frame %d (type=%c, size=%d bytes, format=%d) pts %d key_frame %d [DTS %d]\n",
          pCodecContext->frame_number,
          av_get_picture_type_char(pFrame->pict_type),
          pFrame->pkt_size,
          pFrame->format,
          pFrame->pts,
          pFrame->key_frame,
          pFrame->coded_picture_number
      );

      char frame_filename[1024];
      snprintf(frame_filename, sizeof(frame_filename), "%s-%d.pgm", "frame", pCodecContext->frame_number);
      char png_filename[1024];
      snprintf(png_filename, sizeof(png_filename), "%s-%d.png", "frame", pCodecContext->frame_number);
      // Check if the frame is a planar YUV 4:2:0, 12bpp
      // That is the format of the provided .mp4 file
      // RGB formats will definitely not give a gray image
      // Other YUV image may do so, but untested, so give a warning
      if (pFrame->format != AV_PIX_FMT_YUV420P)
      {
        XCHECK(false);
        fprintf(stderr,"Warning: the generated file may not be a grayscale image, but could e.g. be just the R component if the video format is RGB");
      }
      XINFO << std::endl ;
      // save a grayscale frame into a .pgm file
      XINFO << "Frame meta info xxxx: "
       <<  "linesize? " << pFrame->linesize[0] 
        << "width:" << pFrame->width 
        << "height: " << pFrame->height 
        << "filename : "<<  frame_filename ;
        auto e = now_in_nanoseconds();
        std::string frame_filename_tmp = std::string("./pgms/") + frame_filename;
        std::string png_filename_tmp = std::string("./pgms/") + png_filename;
        //pgm_save(pFrame->data[0], pFrame->linesize[0],
        //         pCodecContext->width, pCodecContext->height, frame_filename_tmp.c_str());
        //frame_2_png(pFrame,png_filename_tmp.c_str());
        XINFO << "single decode cost: " << (e - b)/1000 << std::endl;
        b = now_in_nanoseconds();

        if (scale) {
          do_frame_resize(pFrame, frame_filename_tmp);
        }
        
      frame_count++;
      if (frame_count % 100 ==0 ) {
        auto now = now_in_nanoseconds();
        double diff = (now - last_ts) * 0.000000001;
        
        if (last_ts != -1) {
            float fps = 100.0/diff;
            XINFO << "========= fps: " << fps << std::endl;
        }
        last_ts = now;
        
      }
    }
  }
  return 0;
}

bool HevcDecoderAvcodec::getNextBatchImages(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images) {
   AVPacket *pPacket = av_packet_alloc();
   XCHECK_NOTNULL(pPacket);
    while (av_read_frame(fmt_ctx_, pPacket) >= 0)
    {
      
       
        XCHECK(pPacket->stream_index == 0);
        //auto dec_b = now_in_nanoseconds();
       auto  response = decode_packet(pPacket, codec_context_, frame);
        //auto dec_e = now_in_nanoseconds();
        //XINFO << "single decode cost: " << (dec_e - dec_b)/1000 << std::endl;
        if (response < 0) {
            XERROR << " decode package failed" << std::endl;
            break;
        }
            
    
        
        av_packet_unref(pPacket);
        
    }

    avformat_close_input(&fmt_ctx_);
    av_freep(&io_ctx_);
  return true;
}

bool HevcDecoderAvcodec::getNextBatchImages(std::vector<std::vector<uint8_t>>& batch_rgba) {
  
    return true;
}



int HevcDecoderAvcodec::do_decode_frame(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images) {
   
    
    return 0;
}

} // namespace perception
} // namespace xpilot