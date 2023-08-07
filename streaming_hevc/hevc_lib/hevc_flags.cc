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

#include "hevc_flags.h"
DEFINE_bool(dump_header, false, "whether to dump header when de265");
DEFINE_bool(no_acceleration, false, "wether to acceleration");
DEFINE_bool(enable_logging, true, "wether de265 logging");
DEFINE_int32(verbosity, 0, "verbositiy level for hevc decoder");
DEFINE_bool(dec_in_sep_thread, false, "enable separate decoding thread");
DEFINE_int32(separate_thread_num, 16, "separate decoding threads number");
DEFINE_int32(highest_num, 1000, "highest number of threads");

DEFINE_string(h265_path, 
  "/sandbox/perception/camera_front_fisheye/recording_0_2022-8-4_16:34:13_1659602053954083119.h265", "default h265 path");
DEFINE_string(h265_dir, 
  "/sandbox/perception/tmp_e38_da/out/CameraVideoTopic/camera_front_fisheye", "default h265 dir");

DEFINE_int32(BUFFER_SIZE, 4096, "decode h265 buffer size.");

DEFINE_string(perception_config_path, "/xpilot/perception/config/OTA/Unified_ET1_Guangzhou_ngp.yaml", "global config file.");
// 640*480
DEFINE_string(resize_target_name, "vga","default resize name");
DEFINE_string(h265_out_path, "simple_encode.h265", "default h265 out path");
DEFINE_bool(use_cam_conf, false, "use camera config yaml to controll decoder and resize");
DEFINE_string(process_cam_conf_path, "./cam_process.conf", "path to camera process configuration file(YAML)");