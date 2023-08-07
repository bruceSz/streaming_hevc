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

#include <thread>
#include "infra/common/log.h"
#include "hevc_decoder_context.h"
#include "perception/common/utils/log_utils.h"



extern "C" {
#include <libavutil/imgutils.h>
}


namespace xpilot {
namespace perception {

    
  // @brief Start HEVC publisher service.
  //
  // @returns Whether service starts successfully.
  bool HevcDecoderContext::Start(void) {

    DCHECK_EQ(status_, INITIALIZED);
    status_ = RUNNING;    

    std::ostringstream oss;
    oss << "hevc_decode_x86_" << sensor_id_;

    main_thread_ = std::make_unique<std::thread>(
        &HevcDecoderContext::ThreadOutput, this);

    LOG(INFO) << "HevcDecoderContext Started.";
    return true;

  }

  bool HevcDecoderContext::init_decode_info()  {

    auto ret = stream_context_->Init();  
    if (!ret) {
        return false;
    }
    auto avFormatCtx =  stream_context_->getAVFormatContext();
  
    AVCodec *pCodec = NULL;
    AVCodecParameters *pCodecParameters =  NULL;
    int video_stream_index = -1;
    

    for (int i = 0; i < avFormatCtx->nb_streams; i++)
    {
        AVCodecParameters *pLocalCodecParameters =  NULL;
        pLocalCodecParameters = avFormatCtx->streams[i]->codecpar;
        LOG(INFO) << "AVStream->time_base before open coded" << avFormatCtx->streams[i]->time_base.num 
            << avFormatCtx->streams[i]->time_base.den;
        LOG(INFO) << "AVStream->r_frame_rate before open coded" 
            << avFormatCtx->streams[i]->r_frame_rate.num, avFormatCtx->streams[i]->r_frame_rate.den;
        LOG(INFO) << "AVStream->start_time %" 
            <<  avFormatCtx->streams[i]->start_time;
        LOG(INFO) << "AVStream->duration % " 
            << avFormatCtx->streams[i]->duration;

        LOG(INFO) << "finding the proper decoder (CODEC)";

        AVCodec *pLocalCodec = NULL;

        // finds the registered decoder for a codec ID
        // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga19a0ca553277f019dd5b0fec6e1f9dca
        pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);
        LOG(INFO) << "found the registered decoder: " << pLocalCodecParameters->codec_id;

        XCHECK_NOTNULL(pLocalCodec);
        

        // when the stream is a video we store its index, codec parameters and codec
        if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
            if (video_stream_index == -1) {
                video_stream_index = i;
                pCodec = pLocalCodec;
                pCodecParameters = pLocalCodecParameters;
            }

            LOG(INFO) << "===========Video Codec: resolution " << pLocalCodecParameters->width 
                << "*" <<  pLocalCodecParameters->height ;
        } else if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_AUDIO) {
           XERROR << "Audio stream is skipped.(should not have this codec_type.)" << std::endl;
        }

        // print its name, id and bitrate
        fprintf(stderr,"\tCodec %s ID %d bit_rate %lld\n", pLocalCodec->name, pLocalCodec->id, pLocalCodecParameters->bit_rate);
    }

    XCHECK_NE(video_stream_index, -1);
    LOG(INFO) << "video stream index: " << video_stream_index << std::endl;

    // https://ffmpeg.org/doxygen/trunk/structAVCodecContext.html
    pCodecContext = avcodec_alloc_context3(pCodec);
    XCHECK_NOTNULL(pCodecContext);

    // Fill the codec context based on the values from the supplied codec parameters
    // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#gac7b282f51540ca7a99416a3ba6ee0d16
    ret = avcodec_parameters_to_context(pCodecContext, pCodecParameters) ;
    XCHECK_LE(ret, 0);


    //(*pCodecContext)->thread_count = FLAGS_separate_thread_num;

    pCodecContext->thread_count  = FLAGS_separate_thread_num;
     // Initialize the AVCodecContext to use the given AVCodec.
    // https://ffmpeg.org/doxygen/trunk/group__lavc__core.html#ga11f785a188d7d9df71621001465b0f1d
    if (avcodec_open2(pCodecContext, pCodec, NULL) < 0)
    {
        fprintf(stderr,"failed to open codec through avcodec_open2");
        XCHECK(false);
    }
    // https://ffmpeg.org/doxygen/trunk/structAVFrame.html
    
    LOG(INFO) << "HevcDecoderContext Get decode info done." << std::endl;
    return true;
}

void HevcDecoderContext::ThreadOutput() {

    
    LOG(INFO) << "ioCtx created" << std::endl;
    // it will block here after first feed frame.
    bool res_init = init_decode_info(); 
    if (!res_init) {
        LOG(INFO) << "init decode info failed. return from ThreadOutput" << std::endl;
        return;
    }
    //ret = avformat_find_stream_info(avFormatCtx, NULL);
    auto avFormatCtx = stream_context_->getAVFormatContext();
    //XCHECK_LE(ret, 0);
    //LOG(INFO) << "find stream done." << std::endl;


    if (status_ == STOPPED) {
        LOG(INFO) << "Stop before start." << std::endl;
        return;
    }
    LOG(INFO) << "Enter read frame and loop" << std::endl;
    // TODO while loop here.
    //while(status_ != STOPPED) {
        while (av_read_frame(avFormatCtx, pPacket) >= 0) {
            LOG(INFO) << "Read frame return true." << std::endl;
            if (status_ == STOPPED) {
                LOG(INFO) << "Stop before decode frame." << std::endl;
                break;
            }

          
          //  LOG(INFO) << "pkt data size: " << pPacket->size << " read frame cost: " << (e_read - b_read)/1000 << std::endl;
            // if it's the video stream
            XCHECK(pPacket->stream_index == 0);
            //auto dec_b = xpilot::perception::now_in_nanoseconds();

            auto response = decode_packet(pPacket, pCodecContext, pFrame);
            //auto dec_e = xpilot::perception::now_in_nanoseconds();
            //LOG(INFO) << "single decode cost: " << (dec_e - dec_b)/1000 << std::endl;
            if (response == AVERROR_EOF ) {
                LOG(INFO) << "leaving read frame loop." << std::endl;
                break;
            } else if (response < 0) {
                LOG(ERROR) << " decode package failed, skip this frame." << std::endl;
                //break;
            } 
            // https://ffmpeg.org/doxygen/trunk/group__lavc__packet.html#ga63d5a489b419bd5d45cfd09091cbcbc2
            av_packet_unref(pPacket);
            
        }
        LOG(INFO) << "Leaving while loop, met end of stream? Cleanup decode. "  << std::endl;
        cleanup_decode();
    //}
    status_ = STOPPED;
  return ;

}

void HevcDecoderContext::cleanup_decode() {
    LOG(INFO) << "cleanup decode" << std::endl;
    pPacket->data = nullptr;
    pPacket->size = 0;
    auto res = decode_packet(pPacket, pCodecContext, pFrame);
    if (res == AVERROR_EOF) {
        LOG(INFO) << "cleanup decode done." << std::endl;
        return ;
    } else if (res < 0) {
      XERROR << "last decoder failed."  << std::endl;
      //XCHECK(false);
      return;
    }
    av_packet_unref(pPacket);
    return;
}


  // @brief Initialize HEVC publisher service.
  //
  // @param[in] _cmdline: Parameters from command line or YAML.
  //
  // @returns Whether service is initialized successfully.
  bool HevcDecoderContext::Init() {
    LOG(INFO)  << "HevcDecoderContext init." << std::endl;
    DCHECK_EQ(status_, UNINITIALIZED);
    // Only support fisheye decoder now.

    auto source = std::dynamic_pointer_cast<xpilot::perception::StreamSource>(std::make_shared<BlockingQueueStreamSource>(15));
  
    stream_context_ = std::make_shared<xpilot::perception::MyStreamIOContext>(source);

    // lazy init for stream_context_;
    // stream_context_->Init();
 
    // avFormatCtx =  ioCtx->getAVFormatContext();

    pPacket = av_packet_alloc();
    XCHECK_NOTNULL(pPacket);

    pFrame = av_frame_alloc();
    XCHECK_NOTNULL(pFrame);

     LOG(INFO) << "Init decoder context for sensor:" << sensor_id_  << std::endl;

    auto dst_pair = getScaledSize(msg::SensorID(sensor_id_));
    encoder_->Init(dst_pair.first, dst_pair.second);

   
    resizer_ = std::make_shared<xpilot::perception::HevcResizer>(dst_pair.first, dst_pair.second);

    init_tmp_frame(dst_pair.first, dst_pair.second);
    status_ = INITIALIZED;
    LOG(INFO) << "HevcDecoderContext X86 Init  ok" << std::endl;
    return true;
  }

  void HevcDecoderContext::init_tmp_frame(int width, int height) {

  frame_ = av_frame_alloc();

  frame_->format = xpilot::perception::H265Encoder::FORMAT;
  frame_->width  = width;
  frame_->height = height;

    /* the image can be allocated by any means and av_image_alloc() is
     * just the most convenient way if av_malloc() is to be used */
  auto  ret = av_image_alloc(frame_->data, frame_->linesize, frame_->width, frame_->height,
                         static_cast<AVPixelFormat>(frame_->format), 32);
  XCHECK_GE(ret, 0);
  return;
}


   HevcDecoderContext::~HevcDecoderContext() {
    Stop();
    DeInit();
    if (frame_) {
        av_frame_free(&frame_);
    }
    
    LOG(INFO) << "HevcDecoderContext X86 destroy ok" << std::endl;
   }

    bool HevcDecoderContext::Encode(const AVFrame *frame, AVPacket* pkt) {
        return encoder_->Encode(frame, pkt);
   }

  // @brief Stop HEVC publisher service.
  //
  // @returns Whether service stops successfully.
  bool HevcDecoderContext::Stop() {
    LOG(INFO) << "Stopping decoder: " << std::endl;
    
    
    status_ = STOPPED;
    auto source_ = stream_context_->getSource();
    XCHECK_NOTNULL(source_);

    auto ptr = std::dynamic_pointer_cast<BlockingQueueStreamSource>(source_);
    ptr->Stop();

    
    LOG(INFO) << "HevcDeccoder " << sensor_id_ << " Stop enter";
    if (main_thread_ && main_thread_->joinable()) {
        main_thread_->join();
    }
    LOG(INFO) << "HevcDecoder " << sensor_id_ << " Stop exit";
    return true;
  }

  // @brief De-initialize HEVC publisher service.
  //
  // Step1: Destroy NvMedia HEVC encoder.
  //
  // @returns Whether service is released successfully.
  bool HevcDecoderContext::DeInit(void) {
    if (pPacket != nullptr) {
        av_packet_free(&pPacket);
        pPacket = nullptr;
    }

    if (pFrame != nullptr) {
        av_frame_free(&pFrame);
        pFrame = nullptr;
    }

    if (pCodecContext != nullptr) {
        avcodec_free_context(&pCodecContext);
        pCodecContext = nullptr;
    }
    // not managed by this class, not need to close avFormatCtx
    //auto avFormatCtx =  ioCtx->getAVFormatContext();
    //avformat_close_input(&avFormatCtx);
    return true;
  }

  //std::shared_ptr<AVFrame> GetFrame();

  bool HevcDecoderContext::FeedFrame(const std::shared_ptr<CameraVideo>& _v_frame) {
   // decoder_->FeedFrame(record_ptr->video_frame_);
    Metadata_t meta;
    // for front fisheye
    //LOG(INFO) << "Feed frame for: " << sensor_id_ << "; video cid: " << _v_frame->camera_id()  << std::endl;
    XCHECK_EQ(_v_frame->camera_id(), sensor_id_);
  
    meta.time_stamp = _v_frame->time_stamp().nsec();
    meta.video_frame = _v_frame;
    auto source_ptr = std::dynamic_pointer_cast<BlockingQueueStreamSource>(stream_context_->getSource());
    source_ptr->addCameraVideoFrame(_v_frame);
    meta_queue_.PushBack(meta);
    return true;
  }


int HevcDecoderContext::getBlockingQueueSize()  {
    auto source_ptr = std::dynamic_pointer_cast<BlockingQueueStreamSource>(stream_context_->getSource());
    return source_ptr->getQueueSize();
}

bool HevcDecoderContext::NoWaitingFrame() {
     auto source_ptr = std::dynamic_pointer_cast<BlockingQueueStreamSource>(stream_context_->getSource());
    return source_ptr->isEmpty();
}

int HevcDecoderContext::decode_packet(AVPacket *pPacket, 
    AVCodecContext *pCodecContext, AVFrame *pFrame) {
    static int frame_count = 0;
    static int last_ts = -1;
    
    // Supply raw packet data as input to a decoder
    // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga58bc4bf1e0ac59e27362597e467efff3
    int response = avcodec_send_packet(pCodecContext, pPacket);

    if (response < 0) {
        //Handle this.
        
        Metadata_t t;
        meta_queue_.PopFront(t);
        XERROR << "Error while sending a packet to the decoder: " << response << ". Ts: " << t.time_stamp <<std::endl;
        return response;
    }

    while (response >= 0)
    {
        // Return decoded output data (into a frame) from a decoder
        // https://ffmpeg.org/doxygen/trunk/group__lavc__decoding.html#ga11e6542c4e66d3028668788a1a74217c
        response = avcodec_receive_frame(pCodecContext, pFrame);
        
        if (response == AVERROR(EAGAIN) ) {
            LOG(INFO) << "Need more read." << std::endl;
            break;
        } else if ( response == AVERROR_EOF) {
            LOG(INFO) << "EOF Frame." << std::endl;
            return AVERROR_EOF;
        } else if (response < 0) {

            LOG(INFO) << "Error while receiving a frame from the decoder: " << response ;
            return response;
        }

        LOG(INFO) << "received frame here." << std::endl;

        if (response >= 0) {
            fprintf(stdout,
                "Frame %d sensor %d (type=%c, size=%d bytes, format=%d) pts %d key_frame %d [DTS %d]\n",
                pCodecContext->frame_number, int(sensor_id_),
                av_get_picture_type_char(pFrame->pict_type),
                pFrame->pkt_size,
                pFrame->format,
                pFrame->pts,
                pFrame->key_frame,
                pFrame->coded_picture_number
            );
            
            XCHECK(pFrame->format == AV_PIX_FMT_YUV420P);
            
            
            Metadata_t t;
            meta_queue_.PopFront(t);
            if (callback_) {
                callback_(pFrame, t);
            } else {
                XERROR << "Callback not valid, skip processing frame" << std::endl;
            }
            
        
        }
    }
  return 0;
}

} // namespace perception
} // namespace xpilot