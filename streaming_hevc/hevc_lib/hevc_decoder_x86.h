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

#include <vector>
#include <string>
#include <condition_variable>
#include <deque>
#include <atomic>

#include "libde265/de265.h"
#include "opencv2/opencv.hpp"
#include "camera_service/camera_image.hpp"
#include "camera_service/camera_video.hpp"
#include "msg/SensorID.hpp"
#include "perception/dataProvider/blocking_queue.h"
#include "infra/common/log.h"

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



class ScopedDe265Ctx {
    public:
    static constexpr bool CHECK_HASH = false;
    static constexpr int DISABLE_DEBLOCKING = 0;
    static constexpr int DISABLE_SAO = 0;
    public:
    ScopedDe265Ctx() {
        ctx_ptr_ = de265_new_decoder();

    }
    ~ScopedDe265Ctx() {
        if (ctx_ptr_){
            de265_free_decoder(ctx_ptr_);
        }
        
    }

    void defaultConfig();

    de265_decoder_context* get() {
        return ctx_ptr_;
    }

    private:

    de265_decoder_context* ctx_ptr_ = nullptr;

};


class HevcDecoderAvcodec;
class HevcDecoder {

    public:
        static constexpr int END_OF_BLOCKS = -1;
        static std::shared_ptr<HevcDecoder> create(msg::SensorID cam_id) {
            return std::dynamic_pointer_cast<HevcDecoder>(std::make_shared<HevcDecoderAvcodec>(cam_id));
        }

        HevcDecoder(msg::SensorID cam_id) : cam_id_(cam_id) {}
        virtual ~HevcDecoder () = default;
        virtual void Init() { initialized_ = true; }
        virtual bool Start() { return true; }
        virtual bool sendBlock(const char* block, uint64_t len) = 0;
        virtual bool getNextBatchImages(std::vector<std::vector<uint8_t>>& batch_rgba) = 0;

        //use othere struct later to save memcopy cost.
        virtual bool getNextBatchImages(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images) = 0;
    
    protected:
        msg::SensorID cam_id_;
        bool initialized_ = false;

};



class HevcDecoderDe265: public HevcDecoder {
    public:
    static constexpr std::string_view DECODER_NAME = "de265_hevc_decoder";
    HevcDecoderDe265(msg::SensorID cam_id) : HevcDecoder(cam_id) {}
    ~HevcDecoderDe265() = default;
    void Init() override;

    bool sendBlock(const char* block, uint64_t len) override;

    bool getNextBatchImages(std::vector<std::vector<uint8_t>>& batch_rgba) override;
    bool getNextBatchImages(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images) override;
    

    //bool getNextImage(cv::Mat& img) override ;

    void fill(const de265_image* img, uint8_t* output, int size) ;
    void outputImage(const de265_image* img, std::vector<uint8_t>& output) ;
    private:

    void log_out_warnings();
    
    bool reset_flag_ = false;
    ScopedDe265Ctx  ctx_;
    uint64_t visited_count_ = 0;
    int img_count_ = 0;

};


static inline
void yuv2rgb(uint8_t y, uint8_t u, uint8_t v, uint8_t& r, uint8_t& g, uint8_t& b) {
    int R = std::round(y + 1.403 * (v - 128)); // 四舍五入到最近的整数
    int G = std::round( - 0.343 * (u - 128) - 0.714 * (v - 128));
    int B = std::round(y + 1.770 * (u - 128));


    r = y + 1.402 * (v - 128); // R
      //G
      // algo1: yy - 0.34413 * (vv - 128) - 0.71414 * (uu-128);
      // algo2: Y−0.39465×(U−128)−0.58060×(V−128)
    g = y - 0.34413 * (u-128) - 0.714136 * (v-128);  // G
      // R.
      //  algo1: yy + 1.772 * (vv-128);
      //  algo2: R=Y+1.13983×(V−128);
    b =  y + 1.772*(u-128); //   // B

    return ;
}

class HevcDecoderAvcodec: public HevcDecoder {
  public:

   struct IoBuffer {
        uint8_t* buffer_;
        uint8_t* size_;
        uint8_t* offset_;
   };

/*
    class IOContext {
        private:
            std::string datafile;
            AVIOContext *ioCtx;
            uint8_t *buffer; // internal buffer for ffmpeg
            int bufferSize;
            

        public:
            MyFileIOContext();
            ~MyFileIOContext();
            uint64_t get_size() {
                return fsize_;
            }

            void update_offset(uint64_t s) {
                offset_ += s;
            }
            uint64_t get_offset() {
                return offset_;
            }

            void initAVFormatContext(AVFormatContext *);
            uint64_t fsize_ = -1;
            uint64_t offset_ = 0;

    };*/
    static constexpr char DECODER_NAME[] = "avcodec_hevc_decoder";
    HevcDecoderAvcodec(msg::SensorID cam_id):HevcDecoder(cam_id) {}
    ~HevcDecoderAvcodec() ;

    void Init() override;

    bool sendBlock(const char* block, uint64_t len) override;

    bool getNextBatchImages(std::vector<std::vector<uint8_t>>& batch_rgba) override;
    bool getNextBatchImages(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images) override;
    bool getNextYUV(uint8_t *data[4], int linesize[4],
                           int* width, int* height);
    bool doGetNextBatchImages(std::function<void(AVFrame*)> cb);
    bool getRawFrame(AVFrame** frame);
    bool nextBatchTriggerCB(std::function<void(AVFrame* )>);
   

    bool getNextBatchImages2(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images); 
  private:

    void frame2image(std::shared_ptr<xpilot::msg::camera_service::CameraImage>& img);
  
    int do_decode_frame(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images);
    AVCodec *codec =  nullptr;
    AVCodecContext *codec_context_=  nullptr;
    int64_t frame_count_ = 0;
    AVFrame *frame = nullptr;
    //uint8_t inbuf[INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];
    AVPacket* avpkt;

    AVFormatContext *fmt_ctx_ ;
    AVIOContext *io_ctx_;

};




class BlockingQueueStreamSource: public xpilot::perception::StreamSource {
  public:
    BlockingQueueStreamSource(int count):queue_(count) {}

    void addCameraVideoFrame(std::shared_ptr<xpilot::msg::camera_service::CameraVideo> video_frame) {
        queue_.push(video_frame);
    }

    ~BlockingQueueStreamSource() {
        LOG(INFO) << "BlockingQueueStreamSource destructed. xxxxxxxxxxxx  ----" << std::endl;
    }
    
    int getQueueSize() const {
        return queue_.size();
    }

    bool isEmpty() {
        return getQueueSize() == 0;
    }

    int getNextBlock(char** buf) override {
        // when reset, old video_frame will be released.
        curr_video_frame_.reset();
        //CHECK here.
        int ret = queue_.pop(curr_video_frame_);
        if (ret == -1) {
            LOG(INFO) << "[BlockingQueueStreamSource]Blockingqueue pop failed.(stopped)" << std::endl;
            return 0;
        }

        LOG(INFO) << "Pop next block with size of data: " << curr_video_frame_->data().size()  
            << " queue size: " << queue_.size()
            <<  std::endl;
        

        *buf = reinterpret_cast<char*>(&(curr_video_frame_->data()[0]));
        return curr_video_frame_->data().size();
    }

    void Stop() {
        queue_.clear();
        LOG(INFO) << "HevcblockedQueueSource stopped. xxxxxxxxxxxx" << std::endl;
    }


  private:
    
  std::shared_ptr<xpilot::msg::camera_service::CameraVideo> curr_video_frame_;
  BlockingQueue<std::shared_ptr<xpilot::msg::camera_service::CameraVideo>> queue_;
};

} // namespace perception
} // namespace xpilot