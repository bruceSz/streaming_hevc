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

#include "hevc_camera_config.h"
#include "infra/common/log.h"
#include <yaml-cpp/yaml.h>

namespace xpilot {
namespace perception {

bool CameraHevcConfigMgr::GetHevcCameraConfig(int sensor_id, HevcCameraConfig& ret) {
  if (configs_.find(sensor_id) != configs_.end()) {
    ret =  configs_[sensor_id];
    return true;
  }
  return false;
    
}

std::ostream& operator<<(std::ostream& os, const HevcCameraConfig& c) {
     os << "SensorID id: " << c.sensor_id  << ", camera: " << c.camera_name_ 
        << "src_width: " << c.src_width << "src_height: " << c.src_height << ";"
        << "dst_width: " << c.dst_width << "dst_height: " << c.dst_height << std::endl;

     return os;
  }

void  CameraHevcConfigMgr::LoadYamlConfig(std::string &cfg) {
  
    XCHECK(std::filesystem::exists(cfg));

    YAML::Node root = YAML::LoadFile(cfg);

    YAML::Node camera_config = root["camera_config"];

  for (YAML::const_iterator it = camera_config.begin();
       it != camera_config.end(); ++it) {
        HevcCameraConfig config;
        config.camera_name_ = it->first.as<std::string>();
        config.sensor_id = it->second["sensor_id"].as<int>();
        config.src_width = it->second["src_w"].as<int>();
        config.src_height = it->second["src_h"].as<int>();
      
        config.dst_width = it->second["dst_w"].as<int>();
        config.dst_height = it->second["dst_h"].as<int>();
        configs_[config.sensor_id] = config;
  }

  return ;
}


} // namespace perception
} // namespace xpilot