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

static 
int IOReadFunc(void *data, uint8_t *buf, int buf_size) {
	MyFileIOContext *hctx = (MyFileIOContext*)data;
	
	//NOTE: we open stream in init function,hence remove the check here.
	//XCHECK(hctx->is_initialized());
    if (!buf) {
        XERROR << "Error reading from io context, buf is empty";
        return AVERROR_BUG;
    }
	size_t len = fread(buf, 1, buf_size, hctx->getFileHandler());
    if(len < 0) {
        XERROR << "fread return negative." << std::endl;
        return AVERROR_BUG;
    }
    hctx->update_offset(len);

    XINFO << "read from myFileIOContext: " << buf_size <<
         " offset: " << hctx->get_offset() <<
         "current size: " << hctx->get_size() << std::endl;
	if (len == 0) {
		// Let FFmpeg know that we have reached EOF, or do something else
        XINFO << "end of file" << std::endl;
		return AVERROR_EOF;
	}
	return (int)len;
}

// whence: SEEK_SET, SEEK_CUR, SEEK_END (like fseek) and AVSEEK_SIZE
static 
int64_t IOSeekFunc(void *data, int64_t pos, int whence) {
    XINFO << "IOSeekFunc CALLED" << std::endl;
    MyFileIOContext *hctx = (MyFileIOContext*)data;
	XCHECK(hctx->is_initialized());
	if (whence == AVSEEK_SIZE) {
		// return the file size if you wish to
        //MyFileIOContext *hctx = (MyFileIOContext*)data;
        XINFO << "IO SEEK CALLed" << std::endl;
        return hctx->get_size();

	}
	
	//MyFileIOContext *hctx = (MyFileIOContext*)data;
	int rs = fseek(hctx->getFileHandler(), (long)pos, whence);
	if (rs != 0) {
		return -1;
	}
	long fpos = ftell(hctx->getFileHandler()); // int64_t is usually long long
	return (int64_t)fpos;
}



MyFileIOContext::MyFileIOContext(const std::string &s):initialized_(false) {
    XCHECK(std::filesystem::exists(s));
	datafile.assign(s);
	
	// allocate buffer
	bufferSize = 51200;
	buffer = (uint8_t *)av_malloc(bufferSize); // see destructor for details
	
    fsize_ = std::filesystem::file_size(s);
	// open file
	fh = fopen(datafile.c_str(), "rb");
    XCHECK_NOTNULL(fh);
	
	
	// allocate the AVIOContext
	ioCtx = avio_alloc_context(
				buffer, bufferSize, // internal buffer and its size
				0,            // write flag (1=true, 0=false) 
				(void*)this,  // user data, will be passed to our callback functions
				IOReadFunc, 
				0,            // no writing
				IOSeekFunc
	);
	avFormatCtx_ = avformat_alloc_context();

	avFormatCtx_->pb = ioCtx;
	avFormatCtx_->flags |= AVFMT_FLAG_CUSTOM_IO;

	
}

MyFileIOContext::~MyFileIOContext() {
	if (fh) fclose(fh);
	
	avformat_close_input(&avFormatCtx_);
	av_free(buffer);
	av_free(ioCtx);
}

//TODO: avoid double init.
bool MyFileIOContext::Init() {
	// or read some of the file and let ffmpeg do the guessing
	size_t len = fread(buffer, 1, bufferSize, fh);
	if (len == 0) {
        XINFO << "empty file " << datafile << std::endl;
        return false;
    } 

	fseek(fh, 0, SEEK_SET); // reset to beginning of file
	
	AVProbeData probeData;
	probeData.buf = buffer;
	probeData.buf_size = bufferSize - 1;
	probeData.filename = "";
	//avFormatCtx_->iformat = av_probe_input_format(&probeData, 1);
    XINFO << "init io context done." << std::endl;

	auto fmt_in_ = av_find_input_format("hevc");
	// This will call IOreadFunc.
	auto ret = avformat_open_input(&avFormatCtx_, "", fmt_in_, NULL);

	XINFO << " open av format ret: " << ret << std::endl;
	XCHECK_EQ(ret,0);
	initialized_ = true;

	return true;
}

AVFormatContext* MyFileIOContext::getAVFormatContext() {
	XCHECK(initialized_);
	return avFormatCtx_;
}

}
}


