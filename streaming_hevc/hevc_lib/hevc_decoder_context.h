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

#include <memory>
#include <unordered_map>

#include "hevc_av_io_context.h"
#include "hevc_encoder_x86.h"
#include "hevc_resizer.h"
#include "hevc_flags.h"
#include "hevc_camera_config.h"

#include "camera_service/camera_video.hpp"

#include "msg/SensorID.hpp"
extern "C" {
#include <libavcodec/avcodec.h>
}

namespace xpilot {
namespace perception {


static std::unordered_map<msg::SensorID, std::pair<int, int>> IMAGE_SIZE_MAP = {
    {msg::SensorID::CAMERA_FRONT_MAIN,{3840,2160}},
    {msg::SensorID::CAMERA_FRONT_FISHEYE,{3840,2160}},
    {msg::SensorID::CAMERA_FRONT_LEFT,{1936,1550}},
    {msg::SensorID::CAMERA_FRONT_RIGHT,{1936,1550}},
    {msg::SensorID::CAMERA_REAR_LEFT,{1936,1550}},
    {msg::SensorID::CAMERA_REAR_RIGHT,{1936,1550}},
};


static std::unordered_map<msg::SensorID, std::pair<int, int>> IMAGE_SCALE_MAP = {
    {msg::SensorID::CAMERA_FRONT_MAIN,{2,2}},
    {msg::SensorID::CAMERA_FRONT_FISHEYE,{2,2}},
    {msg::SensorID::CAMERA_FRONT_LEFT,{2,2}},
    {msg::SensorID::CAMERA_FRONT_RIGHT,{2,2}},
    {msg::SensorID::CAMERA_REAR_LEFT,{2,2}},
    {msg::SensorID::CAMERA_REAR_RIGHT,{2,2}},
};

static std::unordered_map<msg::SensorID, std::pair<int,int>> IMAGE_TARGET_SIZE_MAP = {
  {msg::SensorID::CAMERA_FRONT_MAIN,{1920,1080}},
  {msg::SensorID::CAMERA_FRONT_NARROW,{1920,1080}},
  {msg::SensorID::CAMERA_FRONT_FISHEYE,{1920,1080}},
  {msg::SensorID::CAMERA_FRONT_LEFT,{968,774}},
  {msg::SensorID::CAMERA_FRONT_RIGHT,{968,774}},
  {msg::SensorID::CAMERA_REAR_LEFT,{968,774}},
  {msg::SensorID::CAMERA_REAR_RIGHT,{968,774}},
  {msg::SensorID::CAMERA_REAR_MAIN,{914,474}}
};




static inline std::pair<int, int>  get_camera_size(msg::SensorID sensor_id) {
  auto iter = IMAGE_SIZE_MAP.find(sensor_id);
  if (iter != IMAGE_SIZE_MAP.end()) {
    return iter->second;
  }
  //add check here.
  return std::pair<int,int>();
}

static inline
std::pair<int, int> getScaledSize(msg::SensorID sensor_id) {
  if (FLAGS_use_cam_conf) {
    
    HevcCameraConfig conf;
    auto get_cc_succ = CameraHevcConfigMgr::GetInstance().GetHevcCameraConfig(static_cast<int>(sensor_id), conf);
    XCHECK(get_cc_succ);
    return std::pair<int,int>(conf.dst_width, conf.dst_height);
  } else {
    auto iter = IMAGE_TARGET_SIZE_MAP.find(sensor_id);
    auto size_pair = iter->second;
    XCHECK(iter != IMAGE_TARGET_SIZE_MAP.end()); 
    return size_pair;  
  }
  
}

using xpilot::msg::camera_service::CameraVideo;

template<typename T>
msg::SensorID get_camera_id(std::shared_ptr<T> cv_ptr) {
  auto c_id = cv_ptr->camera_id();
  if (c_id == msg::SensorID::CAMERA_FRONT_FISHEYE) {
    c_id = msg::SensorID::CAMERA_FRONT_MAIN;
  }
  return c_id;
}

class HevcDecoderContext {

  public:

template <class T>
struct MetaQueue {
  std::mutex mtx_;
  std::deque<T> queue_;
  void PushBack(const T &t) {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.emplace_back(t);
  }
  bool PopFront(T &ret) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (queue_.empty()) {
        return false;
    }
    ret = queue_.front();
    queue_.pop_front();
    return true;
  }
};

 public:
  struct Metadata_t {
   public:
    long long sys_ts;
    uint64_t time_stamp;                 ///> Frame timestamp
    std::shared_ptr<CameraVideo> video_frame;                   ///> if from file
  };

 public:
  // Delete default constructor
  HevcDecoderContext(void) = delete;

  HevcDecoderContext(msg::SensorID sensor_id)
      : sensor_id_(sensor_id),frame_(nullptr)  {
        encoder_ = std::make_shared<H265Encoder>();
    
      }

  // Destructor
  ~HevcDecoderContext();

  bool isRunning() const {
    return status_ == RUNNING;
  }
  // @brief Start HEVC publisher service.
  //
  // @returns Whether service starts successfully.
  bool Start(void);

  // @brief Initialize HEVC publisher service.
  //
  // @param[in] _cmdline: Parameters from command line or YAML.
  //
  // @returns Whether service is initialized successfully.
  bool Init();

  // @brief Stop HEVC publisher service.
  //
  // @returns Whether service stops successfully.
  bool Stop(void);

  // @brief De-initialize HEVC publisher service.
  //
  // Step1: Destroy NvMedia HEVC encoder.
  //
  // @returns Whether service is released successfully.
  bool DeInit(void);

  AVFrame* getFrame() {
    return frame_;
  }

  std::shared_ptr<AVFrame> GetFrame();

  bool NoWaitingFrame() ;
  bool Encode(const AVFrame *frame, AVPacket* pkt);

  void Resize(const AVFrame* frame, AVFrame* dst_frame) {
    resizer_->Resize(frame, dst_frame);
    return;
  }


    int targetW() {
      return resizer_->targetW();
    }
    int targetH() {
      return resizer_->targetH();
    }
  int getBlockingQueueSize() ;
  

  bool FeedFrame(const std::shared_ptr<CameraVideo>& _v_frame);

  using Callback =
      std::function<bool(const AVFrame* , const Metadata_t &)>;
  void RegisterCallback(const Callback &callback) {
    callback_ = callback;
  }

  

private:
  bool init_decode_info();
  void init_tmp_frame(int width, int height);
  int decode_packet(AVPacket *pPacket, AVCodecContext *pCodecContext, AVFrame *pFrame) ;
  void cleanup_decode();
  void ThreadOutput();
  enum {
    UNINITIALIZED,
    INITIALIZED,
    RUNNING,
    SUSPENDED,
    STOPPED,
    DESTROYED
  } status_ = UNINITIALIZED;
  uint16_t decode_width_ = 0;
  uint16_t decode_height_ = 0;
  uint32_t bitrate_ = 20 * 1000'000;
  int engine_ = 0;

  msg::SensorID sensor_id_ = msg::SensorID::SENSOR_UNKNOWN;
  
  Callback callback_;
  AVPacket * pPacket = nullptr;
  AVFrame* pFrame = nullptr;


  AVFrame*  frame_ = nullptr;
  std::shared_ptr<H265Encoder> encoder_;
  std::shared_ptr<HevcResizer> resizer_;
  AVCodecContext *pCodecContext  = nullptr;
  std::shared_ptr<xpilot::perception::MyStreamIOContext> stream_context_;
  std::unique_ptr<std::thread> main_thread_;
  MetaQueue<Metadata_t> meta_queue_;
};


} // namespace perception
} // namespace xpilot