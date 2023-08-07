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
#include <filesystem>

#include "infra/common/log.h"
#include "hevc_av_io_context.h"

namespace xpilot {
namespace perception {

int MyStreamIOContext::IOReadFunc(void* o, uint8_t *buf, int buf_size) {
	
	MyStreamIOContext *hctx = (MyStreamIOContext*)o;
	if (hctx->isStreamBufferEmpty()) {
		int len = hctx->resetStreamBuffer();
		if (len == 0) {
			LOG(INFO) << "END OF FLAG: " << AVERROR_EOF << std::endl;
			return AVERROR_EOF;
		}
	}
	int len = hctx->consumeBuffer(buf, buf_size);

    //LOG(INFO) << "read from myStreamIOContext: " << buf_size <<
    //     " actual read size: " << len << std::endl;
	
	return (int)len;

}


 int MyStreamIOContext::IOReadFuncTryBest(void* o, uint8_t *buf, int buf_size) {
	
	MyStreamIOContext *hctx = (MyStreamIOContext*)o;
	
	//NOTE: we open stream in init function,hence remove the check here.
	//XCHECK(hctx->is_initialized());

	uint32_t buf_offset = 0;
	size_t left = buf_size;
	XCHECK(left > 0);

	LOG(INFO) << "reading from stream " << left << std::endl;
	//bool left_over = left > 0;
	while(left > 0) {
		LOG(INFO) << "left loop here?" << (left > 0) << std::endl;
		uint32_t len = hctx->consumeBuffer(buf+ buf_offset, left);
		left = left - len;
		buf_offset += len;
		LOG(INFO) << "consumed " << len << " left : " << left  << " len: " << len  << " offset: " << buf_offset << std::endl;
		LOG(INFO) << " now left: " << left << std::endl;
		std::cerr << " now left cerr: " << left << std::endl;
		if (left < 0) {
			XCHECK(false);
		}

		if (left > 0) {
			std::cerr << "inside of left" << std::endl;
			XCHECK(hctx->isStreamBufferEmpty());

			int b_size = hctx->resetStreamBuffer();
			if (b_size == 0 && left == buf_size) {
				// nothing read.
				LOG(INFO) << "END OF FLAG: " << AVERROR_EOF << std::endl;
				return AVERROR_EOF;
			}

			if (b_size == 0 ) {
				LOG(INFO) << "return: " << len << " while desire: " << buf_size << std::endl;
				return buf_size - left;
			}
			//Otherwise, b_size is >0
			XCHECK(b_size > 0);
		} else {
			break;
		}
		

	}

	LOG(INFO) << "io return: " << buf_size << std::endl;

	// default return target buf size.
	return buf_size;
 }


void MyStreamIOContext::Stop() {
	
	LOG(INFO) << "Stop stream io context.  ooooooooooo" << std::endl;
}


MyStreamIOContext::MyStreamIOContext(const std::shared_ptr<StreamSource>& s_)
	:tmp_size_(0), tmp_buffer_(nullptr) {
	
	source_ = s_;
	// allocate buffer
	bufferSize = 51200;
	buffer = (uint8_t *)av_malloc(bufferSize); // see destructor for details

	// allocate the AVIOContext
	
	//////////////
	ioCtx = avio_alloc_context(
				buffer, bufferSize, // internal buffer and its size
				0,            // write flag (1=true, 0=false) 
				(void*)this,  // user data, will be passed to our callback functions
				MyStreamIOContext::IOReadFunc,
				0,            // no writing
				nullptr
	);
	avFormatCtx_ = avformat_alloc_context();
	LOG(INFO) << "After create avformat context: " << avFormatCtx_ << "xxxxxxxxxxxxxxxxxxxxxx" << std::endl;

	avFormatCtx_->pb = ioCtx;
	avFormatCtx_->flags |= AVFMT_FLAG_CUSTOM_IO;
	////////////////

	fmt_in_ = av_find_input_format("hevc");
	LOG(INFO) << "MstreamIOContext created." << std::endl;

}

void MyStreamIOContext::resetFormatCtx() {
	if (ioCtx != nullptr) {
		//av_freep(&ioCtx);

		LOG(INFO) << "buffer:" << buffer << std::endl;
	}
	/*ioCtx = avio_alloc_context(
				buffer, bufferSize, // internal buffer and its size
				0,            // write flag (1=true, 0=false) 
				(void*)this,  // user data, will be passed to our callback functions
				MyStreamIOContext::IOReadFunc,
				0,            // no writing
				nullptr
	);*/
	avFormatCtx_ = avformat_alloc_context();
	LOG(INFO) << "After create avformat context: " << avFormatCtx_ << "xxxxxxxxxxxxxxxxxxxxxx" << std::endl;

	avFormatCtx_->pb = ioCtx;
	avFormatCtx_->flags |= AVFMT_FLAG_CUSTOM_IO;
}

MyStreamIOContext::~MyStreamIOContext() {
	
	LOG(INFO) << "Destrucing MyStreamIOContext." << std::endl;
	if (ioCtx != nullptr) {
		av_freep(ioCtx);
	}
	LOG(INFO) << "Free avformat: " << std::endl;
	if (avFormatCtx_ != nullptr) {
		avformat_close_input(&avFormatCtx_);
		//av_freep(avFormatCtx_);
	}
	LOG(INFO) << "free tmp buffer: " << std::endl;
	if (buffer  != nullptr) {
		//Todo: as the buffer may be used by decoder threads
		//  take care of this.
		//av_freep(buffer);
	}
	

	LOG(INFO) << "Destrucing MyStreamIOContext done." << std::endl;
}

bool MyStreamIOContext::Init() {
	
	// assume source has been initialized. 
	// source->Init();

	LOG(INFO) << "init io context done." << std::endl;
	resetStreamBuffer();

	// This will call IOreadFunc.
	int ret;
	int retry=10;
	for(int i=0;i< retry;i++) {
		ret = avformat_open_input(&avFormatCtx_, "", fmt_in_, NULL);
		if (ret == 0) {
			break;
		}

		LOG(INFO) << "ret is negative, retrying : " << i << "avFormatCtx: " << avFormatCtx_ << std::endl;
		
		if (ret <  0 && i == (retry-1)) {
			XCHECK(avFormatCtx_ == nullptr);
			XERROR << "avforamt open input failed. StreamIOContext return earlier." << std::endl;
			return false;
		}
		resetFormatCtx();
		LOG(INFO) << "New avformat: " << avFormatCtx_ << std::endl;
	}
	
	////if (ret == 0) {
	//	LOG(INFO) << "Loop break output of format open." << std::endl;
		//break;
	//}
	LOG(INFO) << "[stream io context] open av format ret: " << ret << std::endl;
	//}
	

	
	//XCHECK_EQ(ret,0);
	initialized_ = true;
	return true;
}

AVFormatContext* xpilot::perception::MyStreamIOContext::getAVFormatContext() {
	XCHECK(initialized_);
	return avFormatCtx_;
}


}
}


