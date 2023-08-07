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

#include "hevc_decoder_x86.h"
#include "libde265/pps.h"
#include "libde265/image.h"
#include "infra/common/log.h"
#include "gflags/gflags.h"
#include "hevc_flags.h"
#include "opencv2/opencv.hpp"

#include "perception/common/utils/general_utils.hpp"
namespace xpilot {
namespace perception {


void ScopedDe265Ctx::defaultConfig() {
    //XCHECK_N(ctx);
    XCHECK_NOTNULL(ctx_ptr_);
    
    de265_set_parameter_bool(ctx_ptr_, DE265_DECODER_PARAM_BOOL_SEI_CHECK_HASH, CHECK_HASH);
    de265_set_parameter_bool(ctx_ptr_, DE265_DECODER_PARAM_SUPPRESS_FAULTY_PICTURES, false);

    de265_set_parameter_bool(ctx_ptr_, DE265_DECODER_PARAM_DISABLE_DEBLOCKING, DISABLE_DEBLOCKING);
    de265_set_parameter_bool(ctx_ptr_, DE265_DECODER_PARAM_DISABLE_SAO, DISABLE_SAO);

    if (FLAGS_dump_header) {
        de265_set_parameter_int(ctx_ptr_, DE265_DECODER_PARAM_DUMP_SPS_HEADERS, 1);
        de265_set_parameter_int(ctx_ptr_, DE265_DECODER_PARAM_DUMP_VPS_HEADERS, 1);
        de265_set_parameter_int(ctx_ptr_, DE265_DECODER_PARAM_DUMP_PPS_HEADERS, 1);
        de265_set_parameter_int(ctx_ptr_, DE265_DECODER_PARAM_DUMP_SLICE_HEADERS, 1);

    }

        if (FLAGS_no_acceleration) {
        de265_set_parameter_int(ctx_ptr_, DE265_DECODER_PARAM_ACCELERATION_CODE, de265_acceleration_SCALAR);
    }

    if (!FLAGS_enable_logging) {
        de265_disable_logging();
    }
    de265_set_verbosity(FLAGS_verbosity);
    de265_error err = DE265_OK;
    if (FLAGS_dec_in_sep_thread) {
        XINFO << "Hevc decoder with thread: " << FLAGS_separate_thread_num << std::endl;
        err = de265_start_worker_threads(ctx_ptr_, FLAGS_separate_thread_num);
    }
    if (err != DE265_OK) {
        XERROR << "Error starting worker threads: " 
            << FLAGS_separate_thread_num << ", Error: " 
            << std::endl;
    }
            
    de265_set_limit_TID(ctx_ptr_, FLAGS_highest_num);
    XINFO << " DEFAULT CONFIG done " << std::endl;
}

void HevcDecoderDe265::Init() {
    ctx_.defaultConfig();
    de265_reset(ctx_.get());
    XINFO << "DE265 reset done. " << std::endl;
    visited_count_ = 0;
    HevcDecoder::Init();
    XINFO << "HevcDecoderDe265 initialized" << std::endl;
    return ;
}

bool HevcDecoderDe265::sendBlock(const char* block, uint64_t len) {
    XCHECK(initialized_);
    XCHECK_EQ(reset_flag_, false);
    XCHECK_NOTNULL(ctx_.get());
    XCHECK_NOTNULL(block);

    
    if (len == END_OF_BLOCKS) {
        auto err = de265_flush_data(ctx_.get()); // indicate end of stream
        if (err != DE265_OK) {
            XERROR << "Error do de265 final flush" << std::endl;
        }
        reset_flag_ = true;
    } else {
        auto err = de265_push_data(ctx_.get(), block, len, visited_count_, (void*)2);
          if (err != DE265_OK) {
            XERROR << "Error push data to de265 ctx" << std::endl;
            return false;
          }
          visited_count_ += len;
    }
    return true;
}

static void write_picture(const de265_image* img, const std::string& filename)
{



  XINFO << "Writing " << filename << std::endl;
  
 
  
  static FILE* fh = NULL;
  if (fh==NULL) {
    if (strcmp(filename.c_str(), "-") == 0) {
      fh = stdout;
    } else {
      fh = fopen(filename.c_str(), "wb");
    }
  }

  int img_width = de265_get_image_width(img,0);
  int img_height = de265_get_image_height(img,0);

  int stride, cstride;
  const uint8_t* yptr  = de265_get_image_plane(img,0, &stride);
  XCHECK_EQ(stride, img_width);
  // v
  const uint8_t* cbptr = de265_get_image_plane(img,1, &cstride);
  XCHECK_EQ(cstride, img_width/2);
  // u
  const uint8_t* crptr = de265_get_image_plane(img,2, &cstride);
  XCHECK_EQ(cstride, img_width/2);

  
  
  cv::Mat cv_img_mat(img_height, img_width, CV_8UC3,cv::Scalar(0,0,0));
  XINFO << "cv img with size: " << cv_img_mat.rows << ": " << cv_img_mat.cols << std::endl;
  XINFO << "cv img channel: " << cv_img_mat.channels() << std::endl;
  int count = 0;

  de265_chroma chroma = de265_get_chroma_format(img);
  XINFO << "choma is: " << chroma << std::endl;
  // yuv420p
  
  for(int y=0 ;y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      int y_index = x + y* stride;
      uint8_t yy = yptr[y_index];
      

      uint8_t uu = crptr[x/2 + y/2 * cstride];
      uint8_t vv = cbptr[x/2 + y/2 * cstride];
#if(0)
      int x_s = int(x/2)*2;
      int y_s = int(y/2)*2;
      int res_index = 0;
      const uint8_t* t_ptr = nullptr;
      int global_idx = x_s + y_s * stride;
      XCHECK_LT(global_idx, img_width* img_height/2);
      if ((global_idx) >= (img_width * img_height / 4)) {
        res_index = global_idx - (img_width * img_height) / 4;
        t_ptr = crptr;
      } else {
        res_index = global_idx;
        t_ptr = cbptr;
      }
      XCHECK_LT(res_index,(img_width * img_height/4));
      uint8_t uu = t_ptr[res_index];
      uint8_t vv = t_ptr[res_index+1];
#endif
      //uint8_t vv = cbptr[res_index+1];
      //B.
      // algo1: yy + 1.402*(uu-128); 
      // algo2: Y+2.03211×(U−128)
      cv_img_mat.at<cv::Vec3b>(y, x)[2] = yy + 1.402 * (uu- 128); // R
      //G
      // algo1: yy - 0.34413 * (vv - 128) - 0.71414 * (uu-128);
      // algo2: Y−0.39465×(U−128)−0.58060×(V−128)
      cv_img_mat.at<cv::Vec3b>(y, x)[1] = yy - 0.34413 * (vv-128) - 0.714136 * (uu-128);  // G
      // R.
      //  algo1: yy + 1.772 * (vv-128);
      //  algo2: R=Y+1.13983×(V−128);
      cv_img_mat.at<cv::Vec3b>(y, x)[0] =  yy + 1.772*(vv-128); //   // B
      //XINFO << "G of clolor: " << static_cast<uint>(*(color+1)) << std::endl;
      //XCHECK_EQ(color[0],255);
      
      count++;
    }
  }

  XINFO << "total iter cound: " << count << std::endl;

  for (int c=0;c<3;c++) {
    int stride;
    const uint8_t* p = de265_get_image_plane(img, c, &stride);

    int width = de265_get_image_width(img,c);
    int bits = de265_get_bits_per_pixel(img,c);
    int height = de265_get_image_height(img,c);
    XINFO<< "HevcDecoderDe265 bits per pixel: "<< bits
      << "widht: " << width 
      << "height: " << height
      << "stride: " << stride
       << std::endl;

    if (de265_get_bits_per_pixel(img,c)<=8) {
      // --- save 8 bit YUV ---
      
      for (int y=0;y<de265_get_image_height(img,c);y++) {
        fwrite(p + y*stride, width, 1, fh);
      }
    }
    else {
      // --- save 16 bit YUV ---

      int bpp = (de265_get_bits_per_pixel(img,c)+7)/8;
      int pixelsPerLine = stride/bpp;

      uint8_t* buf = new uint8_t[width*2];
      uint16_t* p16 = (uint16_t*)p;

      for (int y=0;y<de265_get_image_height(img,c);y++) {
        for (int x=0;x<width;x++) {
          uint16_t pixel_value = (p16+y*pixelsPerLine)[x];
          buf[2*x+0] = pixel_value & 0xFF;
          buf[2*x+1] = pixel_value >> 8;
        }

        fwrite(buf, width*2, 1, fh);
      }

      delete[] buf;
    }
  }

  fflush(fh);
  std::string jpg_name  = filename+".png";

  //XINFO  << "cv img shape: " << cv_img.shape << std::endl;
  cv::imwrite(jpg_name.c_str(), cv_img_mat);
}

void HevcDecoderDe265::log_out_warnings() {
  XCHECK(initialized_);
    for (;;) {
        de265_error warning = de265_get_warning(ctx_.get());
        if (warning==DE265_OK) {
          break;
        }
        XWARN << "Warning(get next picture): " << de265_get_error_text(warning) << std::endl;
    }
}

  bool HevcDecoderDe265::getNextBatchImages(std::vector<std::shared_ptr<xpilot::msg::camera_service::CameraImage>>& images) {
    XCHECK(initialized_);
    std::vector<std::vector<uint8_t>>  rgba_vec;
    getNextBatchImages(rgba_vec);
    for (auto& rgba_buffer: rgba_vec) {
      std::shared_ptr<xpilot::msg::camera_service::CameraImage> new_ptr= std::make_shared<xpilot::msg::camera_service::CameraImage>();
      new_ptr->data().resize(rgba_buffer.size());
      auto _u_tsc = std::chrono::system_clock::now();
      xpilot::msg::Time time_stamp = xpilot::msg::Time();
      time_stamp.nsec(_u_tsc.time_since_epoch().count());

      new_ptr->time_stamp() = time_stamp;
      new_ptr->height(1080);
      new_ptr->width(1920);
      // TODO: fix this hard code.
      new_ptr->camera_id() = xpilot::msg::SensorID::CAMERA_FRONT_MAIN ;
      new_ptr->format(xpilot::msg::ImageFormat::JPEG);
      new_ptr->size(rgba_buffer.size());
      new_ptr->offset(0);
      images.emplace_back(new_ptr);
    }
    return true;
  }

bool HevcDecoderDe265::getNextBatchImages(std::vector<std::vector<uint8_t>>& batch_rgba) {
  XCHECK(initialized_);
    batch_rgba.clear();
    int more=1;
    XCHECK(initialized_);
      while (more)
        {
          more = 0;
          // decode some more
          auto b_d = now_in_nanoseconds();
          auto err = de265_decode(ctx_.get(), &more);
          //
          

          if (err != DE265_OK) {
            // if (quiet<=1) fprintf(stderr,"ERROR: %s\n", de265_get_error_text(err));

            if (ScopedDe265Ctx::CHECK_HASH && err == DE265_ERROR_CHECKSUM_MISMATCH) {
                XERROR << "Error dec265 hash mismatch err met." << std::endl;
                initialized_ = false;
                reset_flag_ = false;
                return false;
            }
                
            //XERROR << "Error dec265 " << err << "  err met." << std::endl;
            break;
          }

          // show available images
          
          const de265_image* img = de265_get_next_picture(ctx_.get());
          
          

          /*
          if (img->has_pps()) {
            XERROR << "Image has pss. " << std::endl;
            auto xx = img->get_pps();
            //fprintf(stderr,"entropy_coding_sync_enabled_flag: %d\n", img->get_pps().entropy_coding_sync_enabled_flag);  
          } else {
            XERROR << "no pps. " << std::endl;
            auto shared_pps = const_cast<de265_image*>(img)->get_shared_sps();
            XERROR << "shared pps. #" <<shared_pps << "#" <<  std::endl;
          }*/
          
          

          if (img) {
            auto e_d = now_in_nanoseconds(); // time
            XINFO << "De265 decode cost: " << (e_d - b_d)/ 1000 << std::endl;
            b_d = e_d;
            img_count_++;
            std::vector<uint8_t> tmp;
            batch_rgba.push_back(tmp);
#if (0)
            // debug purpose.
            write_picture(img, count_str);
#endif
            auto b_c = now_in_nanoseconds();
            outputImage( img, batch_rgba[batch_rgba.size()-1]);
            auto e_c = now_in_nanoseconds();
             XINFO << "De265 yuv to rgb cost: " << (e_c - b_c)/ 1000 << std::endl;
          }

          // show warnings
          log_out_warnings();
         
        }
        if (reset_flag_) {
            XWARN << "Warning(last call of sendBlock indicate the input has end, set initialzied flag to false " << std::endl;
            initialized_ = false;
        }
        return true;

}


void HevcDecoderDe265::outputImage(const de265_image* img, std::vector<uint8_t>& output) {
  XCHECK(initialized_);
  int img_width = de265_get_image_width(img,0);
  int img_height = de265_get_image_height(img,0);
  XINFO << "height:" << img_height << ";" << "width: " << img_width << std::endl;
  // default is rgba

  
  output.resize(img_width * img_height * 4, 0);

  fill(img, output.data(), output.size());

}
void HevcDecoderDe265::fill(const de265_image* img, uint8_t* output,int size) {
  XCHECK(initialized_);
  int img_width = de265_get_image_width(img,0);
  int img_height = de265_get_image_height(img,0);
  //XINFO << "height:" << img_height << ";" << "width: " << img_width << std::endl;
  // default is rgba

  // default is yuv 420 (maybe 420sp)
  int stride, cstride;
  const uint8_t* yptr  = de265_get_image_plane(img,0, &stride);
  if (stride != img_width) {
    XERROR << "invalid stride SHOULD EQUAL to img width: " << img_width << std::endl;
  }
  XCHECK_EQ(stride, img_width);
  // v
  const uint8_t* cbptr = de265_get_image_plane(img,1, &cstride);
  if (cstride != img_width/2) {
    XERROR << "invalid  cbptr cstride SHOULD EQUAL to img width: " << img_width << std::endl;
  }
  XCHECK_EQ(cstride, img_width/2);
  // u
  const uint8_t* crptr = de265_get_image_plane(img,2, &cstride);
  if (cstride != img_width/2) {
    XERROR << "invalid crptr cstride SHOULD EQUAL to img width: " << img_width << std::endl;
  }

  XCHECK_EQ(cstride, img_width/2);

  std::string yuv_output =  + "./test_de265.yuv";
  FILE* yuv_fh;
  XINFO << "Writing " << yuv_output  << std::endl;
  yuv_fh = fopen(yuv_output.c_str(),"wb");
  XCHECK_NOTNULL(yuv_fh);

  int yuv_len = img_height * img_width + 0.5 * img_height * img_width;
  fwrite(yptr, 1, yuv_len, yuv_fh);

  int count = 0;

  de265_chroma chroma = de265_get_chroma_format(img);
  // yuv420p
  for(int y=0 ;y < img_height; y++) {
    for (int x = 0; x < img_width; x++) {
      int y_index = x + y* stride;
      uint8_t yy = yptr[y_index];
      

      uint8_t uu = crptr[x/2 + y/2 * cstride];
      uint8_t vv = cbptr[x/2 + y/2 * cstride];
#if(0)
      int x_s = int(x/2)*2;
      int y_s = int(y/2)*2;
      int res_index = 0;
      const uint8_t* t_ptr = nullptr;
      int global_idx = x_s + y_s * stride;
      XCHECK_LT(global_idx, img_width* img_height/2);
      if ((global_idx) >= (img_width * img_height / 4)) {
        res_index = global_idx - (img_width * img_height) / 4;
        t_ptr = crptr;
      } else {
        res_index = global_idx;
        t_ptr = cbptr;
      }
      XCHECK_LT(res_index,(img_width * img_height/4));
      uint8_t uu = t_ptr[res_index];
      uint8_t vv = t_ptr[res_index+1];
#endif
      //uint8_t vv = cbptr[res_index+1];
      //B.
      // algo1: yy + 1.402*(uu-128); 
      // algo2: Y+2.03211×(U−128)
      int real_idx = y_index*4; 
      if ((real_idx  + 3) >= size) {
        XERROR << "ERROR computing real_idx: " << real_idx << " size is: " << size << std::endl; 
      }
      XCHECK((real_idx+3) < size);
      
      output[real_idx] = yy + 1.402 * (uu- 128); // R
      //G
      // algo1: yy - 0.34413 * (vv - 128) - 0.71414 * (uu-128);
      // algo2: Y−0.39465×(U−128)−0.58060×(V−128)
      output[real_idx+1] = yy - 0.34413 * (vv-128) - 0.714136 * (uu-128); // G
      // R.
      //  algo1: yy + 1.772 * (vv-128);
      //  algo2: R=Y+1.13983×(V−128);
      output[real_idx+2] =  yy + 1.772*(vv-128); //   // B
      output[real_idx+3] = 1;
      count++;
    }
  }

}

} // namespace perception
} // namespace xpilot