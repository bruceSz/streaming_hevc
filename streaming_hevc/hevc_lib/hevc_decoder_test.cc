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
#include <fstream>
#include <filesystem>
#include <memory>

#define FF_API_WITHOUT_PREFIX 1
#include "gtest/gtest.h"
#include "hevc_decoder_x86.h"
#include "hevc_encoder_x86.h"
#include "hevc_resizer.h"
#include "infra/common/log.h"
#include "gflags/gflags.h"
#include "libde265/de265.h"
#include "hevc_flags.h"
#include "opencv2/opencv.hpp"

#include "hevc_test_common.h"
#include "hevc_av_io_context.h"
#include "hevc_decoder_context.h"
#include "hevc_camera_config.h"

#include "perception/common/utils/general_utils.hpp"
#include "perception/common/utils/cuda_utils.cuh"
#include "perception/common/utils/fps_counter.h"
#include "perception/networks/perception_config_multicam.hpp"
#include "msg/camera_service/camera_video.hpp"
#include "perception/dataProvider/dds_image_mgr.h"
#include "perception/dataProvider/dds_image_source.h"
#include "msg/SensorID.hpp"
#include <xdds.hpp>

#include "dds_consumer.h"

extern "C" {

#include <libavcodec/avcodec.h>
#include <libavutil/parseutils.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>

}

using xpilot::perception::ScopedDe265Ctx;
using xpilot::perception::HevcDecoderDe265;
using xpilot::perception::HevcDecoderAvcodec;
using xpilot::perception::HevcDecoder;
using xpilot::perception::ScopedTimeCounter;
using xpilot::perception::now_in_nanoseconds;
using xpilot::perception::MyFileIOContext;
using xpilot::perception::H265Encoder;
using xpilot::perception::HevcResizer;
using xpilot::perception::HevcDecoderContext;
using xpilot::msg::camera_service::CameraVideo;
using xpilot::perception::EncoderCBAdapter;
using xpilot::perception::DdsConsumer;
using xpilot::perception::ConfigFileParser;
using xpilot::perception::DDSImageSource;
using xpilot::perception::DdsImageManager;
using xpilot::perception::FpsCounter;





DEFINE_string(scale_out_file,"./scale_out_path.h265", "default h265 scale out file path.");
DEFINE_string(config_file, "./ngp.yaml", "configuration file for perception.");

using xpilot::perception::CameraHevcConfigMgr;
using xpilot::perception::HevcCameraConfig;
using xpilot::msg::SensorID;
using std::vector;


template<typename K>
class CounterMap {
  public: CounterMap() = default;
  void update(K key) {
    if (m_.find(key) == m_.end()) {
      m_[key] = 1;
    } else {
      m_[key] += 1;
    }
  }

  int get(K key) {
    if (m_.find(key) == m_.end()) {
      return 0;
    } else {
      return m_[key];
    }
  }
  std::unordered_map<K,int> m_;
};

TEST(ImageSource, testCameraVideo) {


  ConfigFileParser::ParseConfigFile(FLAGS_config_file.c_str());

  xdds::Init();

  
  
  vector<SensorID> sensor_vec = { SensorID::CAMERA_FRONT_LEFT, SensorID::CAMERA_FRONT_RIGHT, SensorID::CAMERA_FRONT_MAIN};

  for (auto id: sensor_vec) {
    DDSImageSource::GetInstance().Init();
    DDSImageSource::GetInstance().RegisterCamera(id);
    DDSImageSource::GetInstance().Start();
    
  }
  
  std::shared_ptr<CounterMap<SensorID>> sensor_msg_cnt_m = std::make_shared<CounterMap<SensorID>>();
  
    std::vector<std::thread> consumeThreads;
    std::unordered_map<SensorID,std::shared_ptr<FpsCounter>> counter_m;
    
    for (auto id: sensor_vec) {
        counter_m[id]  = std::make_shared<FpsCounter>();
        std::thread th([&sensor_msg_cnt_m, id, &counter_m]() {
          while(true) {
            auto sample = DdsImageManager::GetInstance().GetImage(id);
            std::this_thread::sleep_for(std::chrono::milliseconds(17));
            sensor_msg_cnt_m->update(id);
            auto cnt = sensor_msg_cnt_m->get(id);
            counter_m[id]->Update();
            if (cnt % 100 == 0) {
              std::cout << "Met Sensor " << id << " count" << cnt
                << "test FPS:" << counter_m[id]->Get()
                 << std::endl;
            }
          }
        });
        
        
        consumeThreads.emplace_back(std::move(th));
    }

    for(auto& th: consumeThreads) {
      th.join();
    }




}


TEST(HevcDecoderTest, config) {
    auto ins = CameraHevcConfigMgr::GetInstance();

    SensorID main_id = SensorID::CAMERA_FRONT_FISHEYE;
    HevcCameraConfig main_conf;
    auto ret = ins.GetHevcCameraConfig(static_cast<int>(main_id), main_conf);
    EXPECT_TRUE(ret);
    std::cout << main_conf << std::endl;

    SensorID lf_id = SensorID::CAMERA_FRONT_LEFT;
    HevcCameraConfig lf_conf;
    ret = ins.GetHevcCameraConfig(static_cast<int>(lf_id), lf_conf);
    EXPECT_TRUE(ret);

    std::cout << lf_conf << std::endl;

    SensorID lr_id = SensorID::CAMERA_FRONT_RIGHT;
    HevcCameraConfig lr_conf;
    ret = ins.GetHevcCameraConfig(static_cast<int>(lr_id), lr_conf);
    EXPECT_TRUE(ret);
    std::cout << lr_conf << std::endl;


}


TEST(HevcDecoder, avCodecRawApiSendFrame) {
    AVCodec *codec;
    AVCodecContext *c= NULL;
    int frame_count;
    FILE *f;
    AVFrame *frame;
    
    AVPacket avpkt;

    

    av_init_packet(&avpkt);

     /* set end of buffer to 0 (this ensures that no overreading happens for damaged mpeg streams) */
    memset(inbuf + INBUF_SIZE, 0, AV_INPUT_BUFFER_PADDING_SIZE);

    char* outfilename = "./pgms/test%02d4.pgm";
    printf("Decode video file %s to %s\n", FLAGS_h265_path.c_str(), outfilename);

    /* find the mpeg1 video decoder */
    codec = avcodec_find_decoder(AV_CODEC_ID_H265);
    XCHECK_NOTNULL(codec);
    
    
    c = avcodec_alloc_context3(codec);
    XCHECK_NOTNULL(c);

     if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }

    f = fopen(FLAGS_h265_path.c_str(), "rb");
    XCHECK_NOTNULL(f);
    if (!f) {
        fprintf(stderr, "Could not open %s\n", FLAGS_h265_path.c_str());
        exit(1);
    }
    XINFO << "thread count: " << c->thread_count << std::endl;
    c->thread_count  = FLAGS_separate_thread_num;
    frame = av_frame_alloc();
    XCHECK_NOTNULL(frame);
    if (!frame) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }

    frame_count = 0;
    for (;;) {
        avpkt.size = fread(inbuf, 1, INBUF_SIZE, f);
        if (avpkt.size == 0)
            break;

        avpkt.data = inbuf;
        XINFO << "pkt data size: " << avpkt.size << std::endl;
        // if it's the video stream
        
        //auto dec_b = now_in_nanoseconds();
        auto ret = av_packet_from_data(&avpkt, avpkt.data, avpkt.size);
        if (ret < 0) {
          XINFO << "get package from data block failed";
          break;
        }
        auto response = decode_packet(&avpkt, c, frame);
        av_packet_unref(&avpkt);
        //auto dec_e = now_in_nanoseconds();
        //XINFO << "single decode cost: " << (dec_e - dec_b)/1000 << std::endl;
        if (response < 0) {
            XERROR << " decode package failed" << std::endl;
            break;
        }
        //exit(1);
    }

    /* some codecs, such as MPEG, transmit the I and P frame with a
       latency of one frame. You must do the following to have a
       chance to get the last frame of the video */
    avpkt.data = NULL;
    avpkt.size = 0;
    //decode_write_frame(outfilename, c, frame, &frame_count, &avpkt, 1);

    fclose(f);

    avcodec_close(c);
    av_free(c);
    av_frame_free(&frame);
    printf("\n");


}


TEST(HevcDecoder, avCodecRawApi) {


  AVCodec *codec;
    AVCodecContext *c= NULL;
    int frame_count;
    FILE *f;
    AVFrame *frame;
    
    AVPacket avpkt;

    av_init_packet(&avpkt);

    /* set end of buffer to 0 (this ensures that no overreading happens for damaged mpeg streams) */
    memset(inbuf + INBUF_SIZE, 0, AV_INPUT_BUFFER_PADDING_SIZE);

    char* outfilename = "./pgms/test%04d.pgm";
    printf("Decode video file %s to %s\n", FLAGS_h265_path.c_str(), outfilename);

    /* find the mpeg1 video decoder */
    codec = avcodec_find_decoder(AV_CODEC_ID_H265);
    XCHECK_NOTNULL(codec);
    
    
    c = avcodec_alloc_context3(codec);
    XCHECK_NOTNULL(c);
    

    //if(codec->capabilities&AV_CODEC_FLAG_TRUNCATED)
    //    c->flags|= AV_CODEC_FLAG_TRUNCATED; /* we do not send complete frames */

    /* For some codecs, such as msmpeg4 and mpeg4, width and height
       MUST be initialized there because this information is not
       available in the bitstream. */

    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }

    f = fopen(FLAGS_h265_path.c_str(), "rb");
    XCHECK_NOTNULL(f);
    
    XINFO << "thread count: " << c->thread_count << std::endl;
    //c->thread_count  = FLAGS_separate_thread_num;
    frame = av_frame_alloc();
    XCHECK_NOTNULL(frame);
    

    frame_count = 0;
    for (;;) {
        avpkt.size = fread(inbuf, 1, INBUF_SIZE, f);
        if (avpkt.size == 0)
            break;

        /* NOTE1: some codecs are stream based (mpegvideo, mpegaudio)
           and this is the only method to use them because you cannot
           know the compressed data size before analysing it.

           BUT some other codecs (msmpeg4, mpeg4) are inherently frame
           based, so you must call them with all the data for one
           frame exactly. You must also initialize 'width' and
           'height' before initializing them. */

        /* NOTE2: some codecs allow the raw parameters (frame size,
           sample rate) to be changed at any frame. We handle this, so
           you should also take care of it */

        /* here, we use a stream based decoder (mpeg1video), so we
           feed decoder and see if it could decode a frame */
        avpkt.data = inbuf;
        while (avpkt.size > 0) 
            if (decode_write_frame(outfilename, c, frame, &frame_count, &avpkt, 0) < 0) {
             //XERROR << "decode failed. skip this time." << std::endl;
              break;
            }
          
        if (frame_count > 10) {
          break;
        }
                //exit(1);
      }

    /* some codecs, such as MPEG, transmit the I and P frame with a
       latency of one frame. You must do the following to have a
       chance to get the last frame of the video */
    avpkt.data = NULL;
    avpkt.size = 0;
    decode_write_frame(outfilename, c, frame, &frame_count, &avpkt, 1);

    fclose(f);

    avcodec_close(c);
    av_free(c);
    av_frame_free(&frame);
    printf("\n");
}

TEST(HevcDecoder, rawApi) {
  de265_decoder_context* ctx = de265_new_decoder();
  configDe265Ctx(ctx);

  std::ifstream file(FLAGS_h265_path);
  file.seekg(0, std::ios::end);
  int length = file.tellg();
  file.seekg (0, file.beg);
  std::string buff(FLAGS_BUFFER_SIZE,0);

  FILE* fh;
  
  fh = fopen(FLAGS_h265_path.c_str(), "rb");
  

  int offset = 0;
  int read_size = std::min(FLAGS_BUFFER_SIZE, length);

  while( true) {
    //uint8_t buf[FLAGS_BUFFER_SIZE];
    
   file.read(&buff[0], read_size) ;
   EXPECT_TRUE(file);
    //int n = fread(&buff[0],1,read_size,fh);
   // EXPECT_EQ(n, read_size);
    XINFO << "read " << offset << " bytes" << std::endl;
    
    auto err = de265_push_data(ctx, &buff[0], read_size, offset, (void*)2);
    offset += read_size;
    if (err != DE265_OK) {
      XERROR << "push data failed" << std::endl;
      //EXPECT_TRUE(false, "Error push data to ds265 buffer");
      EXPECT_TRUE(false);
      break;
    }

    loop_fetch_image(ctx);
    //XINFO << "h265 read size: " << read_size << " with offset:" << offset << std::endl;
    read_size = std::min(FLAGS_BUFFER_SIZE, length - offset);
    if (read_size <= 0) {
      XINFO << "no further bytes need to be read.[" << read_size << std::endl;
      err = de265_flush_data(ctx); // indicate end of stream
      break;
    }

    if (offset > 9666560) {
      break;
    }
  }
  loop_fetch_image(ctx);

  XINFO << "h265 read size: " << offset << " for: " << FLAGS_h265_path << std::endl;

}



static
void  buf_2_camera_video(const char* buf, int ret,std::shared_ptr<CameraVideo> video_frame) {
   video_frame->data().resize(ret);
    memcpy(&video_frame->data()[0], buf, ret);

    
    auto _u_tsc = std::chrono::system_clock::now();
    xpilot::msg::Time time_stamp = xpilot::msg::Time();
    time_stamp.nsec(_u_tsc.time_since_epoch().count());

    XINFO << "send video frame at:" << time_stamp << std::endl;


    video_frame->time_stamp(time_stamp);
    return;
}



template<typename T>
void decoder_context_read_with(T& reader) {

 
  int byte_read = 0;
  int l_count = 0;
  std::string frame_filename_path = std::string("./pgms/") + "decoder_context.png";

  int dst_w = 3840/2;
  int dst_h = 2160/2;
  HevcResizer resizer(dst_w, dst_h);
  AVPacket pkt;

  AVFrame* dst_frame = nullptr ;  
  create_avframe( dst_w, dst_h, &dst_frame);
  FILE *fh_ = fopen(FLAGS_h265_out_path.c_str(), "wb");

  EncoderCBAdapter encoder([&fh_](const AVPacket* pkt){
    XINFO << "Write pkt xxxxxxxxxxxxxxxx" << std::endl;
     fwrite(pkt->data, 1, pkt->size, fh_);
    return;
  });
  encoder.Init(dst_w, dst_h);
  xpilot::perception::H265Encoder * encoder_ptr = dynamic_cast<xpilot::perception::H265Encoder*>(&encoder);
  
  std::atomic<int> frame_cnt = 0;

  std::atomic<bool> done = false;
  {
    
     
    HevcDecoderContext decoder_context(xpilot::msg::SensorID::CAMERA_FRONT_FISHEYE);
    decoder_context.Init();
    decoder_context.Start();
    decoder_context.RegisterCallback([&frame_filename_path, &dst_frame,&encoder, &resizer, &done, &frame_cnt, &pkt](const AVFrame* frame, const HevcDecoderContext::Metadata_t & meta) -> bool {
      XINFO << "Processing frame at: " << meta.time_stamp << std::endl;
     
      
      XINFO << "Frame raw shape: w:" << frame->width << "; h:" << frame->height <<  std::endl;
      char frame_filename[1024];
      snprintf(frame_filename, sizeof(frame_filename), "./pgms/%s-%d.png", "decode_context-", frame_cnt.load());
      frame_2_png(frame, frame_filename);
      XINFO << "Dump to: " << frame_filename << std::endl;
      frame_cnt +=1;

      //resizer.Resize(frame, dst_frame);
      auto encode_ret = encoder.Encode(frame,&pkt);
    
     //if (pkt.size > 0) {
     //   av_free_packet(&pkt);
     // }
       done = true;
      return true;
    });

    char* buf;
    
    while(true) {
        int ret = reader.getNextBlock(&buf);
        
        if (ret == 0) {
          XINFO << "End of reading h265 file: " << FLAGS_h265_path << std::endl;
          break;
        }

        std::shared_ptr<CameraVideo> video_frame = std::make_shared<CameraVideo>();
        buf_2_camera_video(buf, ret, video_frame);
        decoder_context.FeedFrame(video_frame);

        l_count++;
        byte_read += ret;
    }
    decoder_context.FeedFrame(std::make_shared<CameraVideo>());
    decoder_context.FeedFrame(std::make_shared<CameraVideo>());
    while(!done) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    //std::this_thread::sleep_for(std::chrono::seconds(1));
    decoder_context.Stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    XINFO << "Ddecoder context stopped." << std::endl;
  }
  XINFO << "Total " << byte_read << " bytes read." << " Total real loop count: " << l_count << std::endl;
  encoder.finilize();
   fwrite(xpilot::perception::H265Encoder::ENDCODE, 1, sizeof(xpilot::perception::H265Encoder::ENDCODE), fh_);
  fclose(fh_);
  return;

}


TEST(HevcDecoder, decoder_context_dir_read) {
  H265DirBlockReader reader(FLAGS_h265_dir);
   decoder_context_read_with<H265DirBlockReader>(reader); 
}

TEST(HevcDecoder, decoder_context_read) {
   RawFileBlockReader reader(FLAGS_h265_path);
  decoder_context_read_with<RawFileBlockReader>(reader);
}

TEST(HevcDecoder, blocking_queue) {
  xpilot::perception::BlockingQueue<int> q(10);
  for(int i = 0; i < 10; i++) {
    q.push(i);
  }
  EXPECT_TRUE(q.size() == 10);
  std::thread t1([&q](){
    for(int i=100;i<105;i++) {
      q.push(i);
    }
  });

  std::thread t2([&q](){
    for(int i=0;i<5; i++) {
      int x;
      q.pop(x);
      XINFO << "Pop from bq: " << x << std::endl;
    }
  });
  t1.join();
  t2.join();
  EXPECT_TRUE(q.size() == 10);

  q.clear();
  EXPECT_TRUE(q.size() == 0);

  std::thread t3([&q]() {
    for(int i=0;i<1000;i++) {
      q.push(i);
    }
  });
  int gt = 0;
  for(int i=0;i<1000;i++) {
    gt+=i;
  }

  std::atomic<int> sum= 0;

  std::vector<std::thread> threads;
  for(int t=0; t<10;t++) {

    threads.emplace_back([&q, &sum]() {
      
      int tmp;
      
      while(true) {
        q.pop(tmp);
        sum += tmp;
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        if (q.size() == 0) {
          XINFO << "Sum is:" << sum << std::endl;
          break;
        }
      }
    });
    
  }
  
  for(auto& th: threads) {
    th.join();
  }

  XINFO << "GT is: " << gt << std::endl;

}

TEST(HevcDecoder, AvcodecSingleFileDecodeWork) {
  // simple pass packet.data is not enought.
  singleFileDecodeAvcodecNotWork();
}

TEST(HevcDecoder, AvcodecSingleFileDecode) {
  // not working.
  singleFileDecode<HevcDecoderAvcodec>();
}

TEST(HevcDecoder, 265SingleFileDecode) {
  singleFileDecode<HevcDecoderDe265>();
  
}

TEST(HevcDecoder, decode_example) {
  video_decode_example();
}

TEST(HevcDecoder, avio_read_simple) {
  // this works.
  simpleAvioRead();
}

TEST(HevcDecoder, decode_avformat) {
  // this works.
  decode_using_avformat();
}

TEST(HevcDecoder, avio_io_context_trigger_read) {
  auto reader_ptr = std::dynamic_pointer_cast<xpilot::perception::StreamSource>(std::make_shared<RawFileBlockReader>(FLAGS_h265_path));
  auto trigger_reader = std::dynamic_pointer_cast<xpilot::perception::StreamSource>(std::make_shared<BlockerReaderTrigger>(reader_ptr));

  auto ptr = std::make_shared<xpilot::perception::MyStreamIOContext>(trigger_reader);
  IoContextRead<xpilot::perception::MyStreamIOContext>(ptr);
}

TEST(HevcDecoder, dds_consumer_test) {
  xdds::Init();
  ConfigFileParser::ParseConfigFile(FLAGS_perception_config_path.c_str());
  CUcontext context;
  auto res = xpilot::perception::CudaUtils::CreateCudaDevice(context);
  XCHECK(res);
  auto dds_consumer = std::make_shared<DdsConsumer>("driving", "main", context);

  XINFO << "Create dds consumer done." << std::endl;
  auto initialized_ = dds_consumer->Init();
  XCHECK(initialized_);
  //consumer_->RegisterCallback(cb_);
  dds_consumer->Start();

}

TEST(HevcDecoder, h265_encoder) {
  int i, ret,zx,x,y, got_output;  
  int width = 352;
  int height = 288;
  AVPacket pkt;

  AVFrame* frame = nullptr;
  create_avframe(width,  height, &frame);
  XINFO << "Adapter init done." << std::endl;
  EncoderWriterAdapter encoder(FLAGS_h265_out_path);
  auto init_ret = encoder.Init(width,height);
  XCHECK(init_ret);
  
  XINFO << "Adapter init done." << std::endl;
  
  for (i = 0; i < 25; i++) {
      /* prepare a dummy image */
      /* Y */
      for (y = 0; y < frame->height; y++) {
          for (x = 0; x < frame->width; x++) {
              frame->data[0][y * frame->linesize[0] + x] = x + y + i * 3;
          }
      }
      /* Cb and Cr */
      for (y = 0; y < frame->height/2; y++) {
          for (x = 0; x < frame->width/2; x++) {
              frame->data[1][y * frame->linesize[1] + x] = 128 + y + i * 2;
              frame->data[2][y * frame->linesize[2] + x] = 64 + x + i * 5;
          }
      }
      frame->pts = i;
      auto encode_ret = encoder.Encode(frame,&pkt);
      if (pkt.size > 0) {
        av_free_packet(&pkt);
      }
      
  }

  /* add sequence end code to have a real mpeg file */
  av_freep(&frame->data[0]);
  av_frame_free(&frame);
}

TEST(HevcDecoder, encode_simple) {
  video_encode_example("test.h265", AV_CODEC_ID_H265);
}

TEST(HevcDecoder, h265_resize_e2e) {
  auto reader_ptr = std::dynamic_pointer_cast<xpilot::perception::StreamSource>(std::make_shared<RawFileBlockReader>(FLAGS_h265_path));
  auto ioCtx = std::make_shared<xpilot::perception::MyStreamIOContext>(reader_ptr);
  
  ioCtx->Init();
  int width = 3840;
  int height = 2160;
  int dst_w = width/2;
  int dst_h = height/2;

  AVPacket pkt;

  AVFrame* dst_frame = nullptr ;  
  create_avframe( dst_w, dst_h, &dst_frame);
   HevcResizer resizer(dst_w, dst_h);

  auto avFormatCtx =  ioCtx->getAVFormatContext();

  EncoderWriterAdapter encoder(FLAGS_h265_out_path);
  encoder.Init(dst_w, dst_h);
  xpilot::perception::H265Encoder * encoder_ptr = dynamic_cast<xpilot::perception::H265Encoder*>(&encoder);
  
  AVFrame* pFrame = nullptr;
  AVPacket * pPacket = nullptr;
  int video_stream_index = -1;
  AVCodecContext *pCodecContext;
  get_decode_info( avFormatCtx,  &pPacket,  &pFrame, &video_stream_index, &pCodecContext ); 
    
  pCodecContext->thread_count  = FLAGS_separate_thread_num;
  

  auto cb = [&dst_frame, &pkt, &resizer, & encoder](AVFrame* frame) {
    XINFO << "Processing frame: " << std::endl;
    resizer.Resize(frame, dst_frame);
    auto encode_ret = encoder.Encode(frame,&pkt);
    
    if (pkt.size > 0) {
      av_free_packet(&pkt);
    }
   

  };

  XINFO << "Enter read frame and loop" << std::endl;
  while (av_read_frame(avFormatCtx, pPacket) >= 0) {
      
      // if it's the video stream
      EXPECT_TRUE(pPacket->stream_index == video_stream_index);
      //auto dec_b = xpilot::perception::now_in_nanoseconds();
      

      auto response = decode_packet_cb(pPacket, pCodecContext, pFrame, cb );
      //auto dec_e = xpilot::perception::now_in_nanoseconds();
      //XINFO << "single decode cost: " << (dec_e - dec_b)/1000 << std::endl;
      if (response < 0) {
          XERROR << " decode package failed" << std::endl;
          break;
      }
      // https://ffmpeg.org/doxygen/trunk/group__lavc__packet.html#ga63d5a489b419bd5d45cfd09091cbcbc2
      av_packet_unref(pPacket);
      
  }
  XINFO << "cleanup decode. Encode back: "  << std::endl;
  cleanup_decode_cb(pPacket, pCodecContext, pFrame, cb);
  //avformat_close_input(&avFormatCtx);
  
  return;
  
}

TEST(HevcDecoder, scale_simple) {

  AVFrame * frame; 
  AVFrame* dst_frame; 
  
  int width = 352;
  int height = 288;
  create_avframe( width,  height, &frame);

  int test_img_num = 30;

  int dst_w = 352/2;
  int dst_h = 288/2;
  create_avframe( dst_w, dst_h, &dst_frame);

  HevcResizer resizer(dst_w, dst_h);


  
  for (int i = 0; i < test_img_num; i++) {
      /* prepare a dummy image */
      /* Y */
      for (int y = 0; y < frame->height; y++) {
          for (int x = 0; x < frame->width; x++) {
              frame->data[0][y * frame->linesize[0] + x] = x + y + i * 3;
          }
      }
      /* Cb and Cr */
      for (int y = 0; y < frame->height/2; y++) {
          for (int x = 0; x < frame->width/2; x++) {
              frame->data[1][y * frame->linesize[1] + x] = 128 + y + i * 2;
              frame->data[2][y * frame->linesize[2] + x] = 64 + x + i * 5;
          }
      }
      frame->pts = i;
      char frame_filename[1024];
      char frame_filename_sz[1024];
      snprintf(frame_filename, sizeof(frame_filename), "%s-%d.png", "frame", i);
      snprintf(frame_filename_sz, sizeof(frame_filename_sz), "%s-%d_rz.png", "frame", i);
      std::string frame_filename_path = std::string("./pgms/") + frame_filename;
      std::string frame_filename_path_sz = std::string("./pgms/") + frame_filename_sz;
      frame_2_png(frame, frame_filename_path.c_str());
      resizer.Resize(frame, dst_frame);
      frame_2_png(dst_frame, frame_filename_path_sz.c_str());
  }

  av_freep(&frame->data[0]);
  av_frame_free(&frame);

  if (dst_frame->data[0] != nullptr) {
    //TODO： fix this , will crash/ (double free).
    //av_freep(dst_frame->data[0]);
    
  }
  
  //av_frame_free(&dst_frame);

}
TEST(HevcDecoder, av_stream_io_context_dir_read) {
  auto reader_ptr = std::dynamic_pointer_cast<xpilot::perception::StreamSource>(std::make_shared<H265DirBlockReader>(FLAGS_h265_dir));
 
  auto ptr = std::make_shared<xpilot::perception::MyStreamIOContext>(reader_ptr);
  IoContextRead<xpilot::perception::MyStreamIOContext>(ptr);
}

TEST(HevcDecoder, av_stream_io_context_file_read) {
  auto reader_ptr = std::dynamic_pointer_cast<xpilot::perception::StreamSource>(std::make_shared<RawFileBlockReader>(FLAGS_h265_path));
 
  auto ptr = std::make_shared<xpilot::perception::MyStreamIOContext>(reader_ptr);
  IoContextRead<xpilot::perception::MyStreamIOContext>(ptr);
}

TEST(HevcDecoder, av_stream_io_context_file_read_encode_bak) {
  auto reader_ptr = std::dynamic_pointer_cast<xpilot::perception::StreamSource>(std::make_shared<RawFileBlockReader>(FLAGS_h265_path));
 
  auto ptr = std::make_shared<xpilot::perception::MyStreamIOContext>(reader_ptr);
  IoContextRead<xpilot::perception::MyStreamIOContext>(ptr, true);
}

TEST(HevcDecoder, av_io_context_read) {
  auto ptr = std::make_shared<MyFileIOContext>(FLAGS_h265_path);
  IoContextRead<xpilot::perception::MyFileIOContext>(ptr);
}

TEST(HevcDecoder, dirFilesDecode){

  EXPECT_TRUE(std::filesystem::exists(FLAGS_h265_dir));
  std::filesystem::path p_obj(FLAGS_h265_dir);
  std::filesystem::directory_entry entry(p_obj);
  std::filesystem::directory_iterator list(p_obj);
  EXPECT_EQ(entry.status().type(), std::filesystem::file_type::directory);
  //XINFO << "DIR NAME:" << entry.status().type() << std::endl;

  std::string buff(FLAGS_BUFFER_SIZE,0);
  //XINFO << buff << std::endl; 
  HevcDecoderDe265 decoder(xpilot::msg::SensorID::CAMERA_FRONT_MAIN);
  decoder.Init();

  auto proc_fn = [&buff, &decoder] (std::string path) {
    XINFO << "processing file name: " << path << std::endl;

    std::ifstream file(path);
    EXPECT_TRUE(file);
    file.seekg(0, file.end);
    int length = file.tellg();
    file.seekg(0, file.beg);

    int read_size = std::min(FLAGS_BUFFER_SIZE, length);
    int offset = 0;
    while(read_size > 0  && file.read(&buff[0], read_size) ) {
        auto res = decoder.sendBlock(&buff[0], read_size);
        EXPECT_TRUE(res);
        std::vector<std::vector<uint8_t>> images;
        res = decoder.getNextBatchImages(images);
        EXPECT_TRUE(res);
        if (res) {
          if (images.size() > 0) {
            XINFO << "decoded images: " << images.size() << std::endl;
          }
        }

        offset += read_size;
        read_size = std::min(FLAGS_BUFFER_SIZE, length - offset);
    }

  };

  //std::unordered_map<std::string, std::string> name2path;
  std::vector<std::string> names;
  std::vector<uint64_t> ts_list;
  std::vector<std::string> paths;
  for(auto& it: list) {
    std::string name = it.path().filename();
    std::string suffix = ".h265";
    if (0 == name.compare(name.length() - suffix.length(), suffix.length(), suffix)) { 
      std::filesystem::path full_path = p_obj / it.path();

      std::string stem =  it.path().stem();
      std::string ts(stem.substr(stem.rfind("_")+1));
      ts_list.push_back(std::stoll(ts));
      paths.push_back(full_path.lexically_normal());
    }
  }  
  for(auto &t: ts_list) {
    XINFO << "ts is: " << t << std::endl;
  }
  std::vector<size_t> idx(ts_list.size());
  std::iota(idx.begin(), idx.end(), 0);
  stable_sort(idx.begin(), idx.end(),
       [&ts_list](size_t i1, size_t i2) {return ts_list[i1] < ts_list[i2];});

  for (auto i: idx) {
    //XINFO  << "idx is: " << i << " ts is: " << paths[i] << std::endl;
    //paths[i]
    XINFO << "idx is: " << i;
    proc_fn(paths[i]);
  }

}

TEST(HevcDecoder, H265DirBlockReader) {
  H265DirBlockReader reader(FLAGS_h265_dir);
  char* buf;
  int total = 0;
  while(true) {
    int ret = reader.getNextBlock(&buf);
    if (ret == 0) {
      XINFO << "end of dir h265 files process. Total:" << total << std::endl;
      break;
    }
    total++;
  }
  
}

TEST(HevcDecoder, getFileBlocks) {
  RawFileBlockReader reader(FLAGS_h265_path);

  char* buf;
  int byte_read = 0;
  while(true) {
    int ret = reader.getNextBlock(&buf);
    if (ret == 0) {
      XINFO << "End of reading h265 file: " << FLAGS_h265_path << std::endl;
      break;
    }
    byte_read += ret;
  }
  XINFO << "Total " << byte_read << " bytes read." << std::endl;
  return;
}

TEST(HevcDecoder, scaleTestNotWorking) {
    uint8_t *src_data[4], *dst_data[4];
    int src_linesize[4], dst_linesize[4];
    int src_w = 1920, src_h = 1080;
    int dst_w, dst_h;
    
    enum AVPixelFormat src_pix_fmt = AV_PIX_FMT_YUV420P, dst_pix_fmt = AV_PIX_FMT_YUV420P;
    // name for 1920 * 1080
    const char *dst_size = "hd1080";
    const char *dst_filename = FLAGS_scale_out_file.c_str();

    std::ifstream file(FLAGS_h265_path);
    XINFO << "Scale test on: " << FLAGS_h265_path << std::endl;
    EXPECT_TRUE(file);
    file.seekg(0, file.end);
    int length = file.tellg();
    file.seekg(0, file.beg);


   
    FILE *dst_file;
    int dst_bufsize;
    struct SwsContext *sws_ctx;
    int i, ret;

    ret = av_parse_video_size(&dst_w, &dst_h, dst_size) ;
    XCHECK_GE(ret, 0);
    dst_file = fopen(dst_filename, "wb");
    XCHECK_NOTNULL(dst_file);


    AVCodecContext* c;
    
    AVCodec *codec;
    
    create_context(AV_CODEC_ID_HEVC, &codec, &c) ;

    /* put sample parameters */
    c->bit_rate = 400000;
    /* resolution must be a multiple of two */
    c->width = dst_w;
    c->height = dst_h;
    /* frames per second */
    c->time_base = (AVRational){1,25};
    /* emit one intra frame every ten frames
     * check frame pict_type before passing frame
     * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
     * then gop_size is ignored and the output of encoder
     * will always be I frame irrespective to gop_size
     */
    c->gop_size = 10;
    c->max_b_frames = 1;
    c->pix_fmt = dst_pix_fmt;


    /* open it */
    ret = avcodec_open2(c, codec, NULL);
    XCHECK_GE(ret, 0);


    /* create scaling context */
    sws_ctx = sws_getContext(src_w, src_h, src_pix_fmt,
                             dst_w, dst_h, dst_pix_fmt,
                             SWS_BILINEAR, NULL, NULL, NULL);
    XCHECK_NOTNULL(sws_ctx);

    /* allocate source and destination image buffers */
    ret = av_image_alloc(src_data, src_linesize,src_w, src_h, src_pix_fmt, 16);
        

    XCHECK_GE(ret, 0);

    /* buffer is going to be written to rawvideo file, no alignment */
    ret = av_image_alloc(dst_data, dst_linesize,
                              dst_w, dst_h, dst_pix_fmt, 16);

    XCHECK_GE(ret, 0);
    dst_bufsize = ret;


     
    HevcDecoderAvcodec decoder( xpilot::msg::SensorID::CAMERA_FRONT_MAIN);
    decoder.Init();

    std::string buff(FLAGS_BUFFER_SIZE+ AV_INPUT_BUFFER_PADDING_SIZE,0);
    int read_size = std::min(FLAGS_BUFFER_SIZE, length);
    int offset = 0;
    int loop_count = 0;
    
    AVPacket pkt;
    AVFrame* frame = av_frame_alloc();
    XCHECK_NOTNULL(frame);

    while( read_size > 0 && file.read(&buff[0], read_size)) {
          
          {
           // ScopedTimeCounter tsend("send_block");
            decoder.sendBlock(&buff[0], read_size);
          }

          offset += read_size;
          read_size = std::min(FLAGS_BUFFER_SIZE, length - offset);

          
          {
            //ScopedTimeCounter tgetNext("getNextbatchImage");
            auto res = decoder.getNextYUV(src_data, src_linesize, &src_w, &src_h);
            //AVFrame* frame = nullptr;
            //auto res = decoder.getRawFrame(&frame);
            EXPECT_TRUE(res);
            XINFO << "YUV shape: w:" << src_w << " h:" << src_h << std::endl;
            XINFO << "YUV resize shape: w:" << dst_w << " h:" << dst_h << std::endl;
          }
       

        /* convert to destination format */
        sws_scale(sws_ctx, (const uint8_t * const*)src_data,
                  src_linesize, 0, src_h, dst_data, dst_linesize);

        
        for(int i=0 ;i < 4; i++) {
          frame->data[i] = src_data[i];
          frame->linesize[i] = src_linesize[i];
        }
        
        frame->width = src_w;
        frame->height = src_h;
        frame->format = AV_PIX_FMT_YUV420P;
        
        {
          /* write scaled image to file */
          //fwrite(dst_data[0], 1, dst_bufsize, dst_file);
          
          /* encode the image */
          int got_output;
         
          //auto res = decoder.getRawFrame(&frame);
     
          
          XCHECK_NOTNULL(frame);
          XINFO << "Frame shapre: " << frame->width << ":" << frame->height << std::endl;

          av_init_packet(&pkt);
          pkt.data = NULL;    // packet data will be allocated by the encoder
          pkt.size = 0;
            ret = avcodec_encode_video2(c, &pkt, frame, &got_output);
          if (ret < 0) {
              fprintf(stderr, "Error encoding frame\n");
              exit(1);
          }

          if (got_output) {
              printf("Write frame %3d (size=%5d)\n", loop_count, pkt.size);
              fwrite(pkt.data, 1, pkt.size, dst_file);
              av_free_packet(&pkt);
          }
        }
      
          loop_count++;

          if (loop_count > 10) {
            break;
          }
      }

   

    fclose(dst_file);
    // src_data is alloced in decoder.
    // hence no need to free it.
    //av_freep(&src_data[0]);
    av_freep(&dst_data[0]);
    sws_freeContext(sws_ctx);
    return ;

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  XINFO << "before run all tests. " << FLAGS_separate_thread_num << std::endl;
  return RUN_ALL_TESTS();
}