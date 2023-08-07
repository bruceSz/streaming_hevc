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

#include <cstdio>
#include <string>

extern "C" {

//#include <libavcodec/avcodec.h>
//#include <libavformat/avio.h>
#include <libavformat/avformat.h>
//#include "libavutil/cpu.h"
//#include "libavutil/parseutils.h"
//#include <libswscale/swscale.h>
//#include <libavutil/imgutils.h>

}

#include "hevc_decoder_x86.h"



namespace xpilot {
namespace perception {



class  MyIOContext {
	public:
	virtual bool Init() = 0;
	virtual AVFormatContext * getAVFormatContext() = 0;
	protected:
	
};

class MyFileIOContext : public MyIOContext {


public:
	MyFileIOContext(const std::string &datafile);
	~MyFileIOContext();
	uint64_t get_size() {
		return fsize_;
	}

	bool Init() override;

	void update_offset(uint64_t s) {
		offset_ += s;
	}
	uint64_t get_offset() {
		return offset_;
	}
	FILE* getFileHandler()  {
		return fh;
	}
	bool is_initialized() {
		return initialized_;
	}

	AVFormatContext * getAVFormatContext() override;

  private:
	uint64_t fsize_ = -1;
	uint64_t offset_ = 0;
	std::string datafile;
	AVIOContext *ioCtx;
	uint8_t *buffer; // internal buffer for ffmpeg
	int bufferSize;
	FILE *fh;
	AVFormatContext *avFormatCtx_;
	bool initialized_;
	
};



class MyStreamIOContext: public MyIOContext{

	
	

public:
	MyStreamIOContext(const std::shared_ptr<StreamSource>& s_);
	~MyStreamIOContext();	

	// drain from source.
	void drain();
	void Stop();

	bool Init() override;
	AVFormatContext * getAVFormatContext() override ;

	bool isStreamBufferEmpty() {
	
		return tmp_size_ <= 0;
	}

	int resetStreamBuffer() {
		XCHECK(source_);
		LOG(INFO) << "reset stream buffer" << std::endl;
		if (met_end_) {
			LOG(INFO) << "met end before, return 0 block size" << std::endl;
			return 0;
		}
		tmp_size_ = source_->getNextBlock(&tmp_buffer_);
		if (tmp_size_ == 0) {
			met_end_ = true;
		}
		
		total_stream_ += tmp_size_;
		// for online case, this should not happen.
		return tmp_size_;
	}

	int consumeBuffer(uint8_t *buf, int target_size) {
		// make sure user of this method call size check first.
		//XINFO << "tmp_size_: " << tmp_size_ << std::endl; 
		XCHECK(tmp_size_ >= 0);
		if (tmp_size_ == 0) {
			return 0;
		}
		
		int read_size = std::min(target_size, tmp_size_);
		// consume
		memcpy(buf, tmp_buffer_, read_size);
		tmp_buffer_ = tmp_buffer_ + read_size;
		tmp_size_ -= read_size;
		//XINFO << "left: " << tmp_size_ << std::endl;

		total_consumed_ += read_size;
		return read_size;

	}
	std::shared_ptr<StreamSource> getSource() { return source_;}

private:
	
	void resetFormatCtx();
	static int IOReadFunc(void* o, uint8_t *buf, int buf_size);
	static int IOReadFuncTryBest(void* o, uint8_t *buf, int buf_size);
	
	AVFormatContext *avFormatCtx_;
	AVInputFormat * fmt_in_;
	bool initialized_;
	uint64_t total_consumed_ = 0;
	uint64_t total_stream_ = 0;
	std::shared_ptr<StreamSource> source_;
	AVIOContext *ioCtx;
	uint8_t *buffer; // internal buffer for ffmpeg
	int bufferSize;  // internal buffer size for ffmpeg to use.
	char* tmp_buffer_;
	int tmp_size_;
	bool met_end_ = false;
};


} // namespace perception
} // namespace xpilot




