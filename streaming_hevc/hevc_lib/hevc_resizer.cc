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

#include "infra/common/log.h"
#include "hevc_resizer.h"


extern "C" {
#include <libswscale/swscale.h>
}

namespace xpilot {
namespace perception {


void HevcResizer::Resize(const AVFrame* frame, AVFrame* dst_frame) {
        
    XCHECK_NOTNULL(frame);
    int src_w = frame->width;
    int src_h = frame->height;
    enum AVPixelFormat src_pix_fmt = static_cast<enum AVPixelFormat>(frame->format);
    enum AVPixelFormat dst_pix_fmt = src_pix_fmt;
    /* create scaling context */
    auto sws_ctx = sws_getContext(src_w, src_h, src_pix_fmt,
                                w_, h_, dst_pix_fmt,
                                SWS_BILINEAR, NULL, NULL, NULL);
    XCHECK(dst_frame != nullptr);
    
    dst_frame->width = w_;
    dst_frame->height = h_;
    dst_frame->format = dst_pix_fmt;

    

    int ret = sws_scale(sws_ctx, (const uint8_t * const*)frame->data,
                  frame->linesize, 0, frame->height, dst_frame->data, dst_frame->linesize);
    XCHECK_GE(ret, 0);
    return;
}

} // namespace perception
} // namespace xpilot
