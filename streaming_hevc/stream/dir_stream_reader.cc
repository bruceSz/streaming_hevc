

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

#include <algorithm>
#include <filesystem>

#include "streaming_hevc/stream/dir_stream_reader.h"

namespace streaming {
namespace hevc {

H265DirBlockReader::H265DirBlockReader(std::string fpath)
    : dir_path_(std::move(fpath)), curr_idx_(0), buffer_(BUFFER_SIZE, '0') {
  XCHECK(std::filesystem::exists(dir_path_));
  std::filesystem::path p_obj(dir_path_);
  std::filesystem::directory_entry entry(p_obj);
  std::filesystem::directory_iterator list(p_obj);
  EXPECT_EQ(entry.status().type(), std::filesystem::file_type::directory);

  std::vector<int64_t> ts_list;
  for (auto& it : list) {
    std::string name = it.path().filename();

    std::string suffix = ".h265";
    if (0 == name.compare(name.length() - suffix.length(), suffix.length(),
                          suffix)) {
      XINFO << "PROC file: " << name << std::endl;
      std::filesystem::path full_path = p_obj / it.path();

      std::string stem = it.path().stem();
      std::string ts(stem.substr(stem.rfind("_") + 1));
      ts_list.push_back(std::stoll(ts));
      file_paths_.emplace_back(full_path.lexically_normal());
    }
  }

  ts_ordered_path_idxs_ = std::vector<int>(ts_list.size());
  std::iota(ts_ordered_path_idxs_.begin(), ts_ordered_path_idxs_.end(), 0);
  stable_sort(
      ts_ordered_path_idxs_.begin(), ts_ordered_path_idxs_.end(),
      [&ts_list](size_t i1, size_t i2) { return ts_list[i1] < ts_list[i2]; });
}

int H265DirBlockReader::getNextBlock(char** buf) {
  //
  if (curr_idx_ >= file_paths_.size()) {
    return 0;
  } else {
    int curr_path_idx = ts_ordered_path_idxs_[curr_idx_++];
    std::string fpath = file_paths_[curr_path_idx];
    fh_.close();
    XINFO << "OPEN h265 file: " << fpath << std::endl;
    fh_.open(fpath);
    XCHECK(fh_);
    int total_f_sz = std::filesystem::file_size(fpath);
    if (total_f_sz > buffer_.size()) {
      XINFO << "Resize buffer to: " << buffer_.size() << std::endl;
      buffer_.resize(total_f_sz);
    }

    XINFO << "Reading file: " << fpath << " with size: " << total_f_sz
          << std::endl;
    XCHECK(fh_.read(&buffer_[0], total_f_sz));
    *buf = &buffer_[0];
    return total_f_sz;
  }
}

}  // namespace hevc
}  // namespace streaming
   // Path: streaming_hevc/stream/dir_stream_reader.cc