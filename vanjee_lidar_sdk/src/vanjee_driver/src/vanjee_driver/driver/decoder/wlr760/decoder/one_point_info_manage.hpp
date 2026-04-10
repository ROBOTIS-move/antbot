/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <cassert>
#include <vector>

// #include <vanjee_driver/driver/decoder/decoder.hpp>
#include <vanjee_driver/driver/decoder/trigon.hpp>

#define MAX_COL_NUM_760 936
vanjee::lidar::Trigon trigon760_;

typedef struct _OnePointInfo760 {
  unsigned int index = 0;
  // unsigned int cluster_index = 0;
  unsigned short rotating_mirror_id = 0;
  unsigned short motor_speed = 0;
  unsigned char device_model = 0;
  unsigned char echo_type = 0;
  short hor_azimuth = 0;
  short ver_azimuth = 0;
  double hor_angle = 0;
  double ver_angle = 0;

  unsigned short hor_angle_index = 0;

  float x = 0;
  float y = 0;
  float z = 0;
  double distance = 0;
  unsigned char reflectivity = 0;
  unsigned char intensity = 0;
  unsigned char confidence = 0;
  unsigned short line_id = 0;
  double timestamp = 0;

  int non_filterable = 0;

  void calculatexyz() {
    int32_t v_angle = (int32_t(ver_angle * 1000) + 360000) % 360000;
    int32_t h_angle = (int32_t(hor_angle * 1000) + 360000) % 360000;
    float xy = distance * trigon760_.cos(v_angle);
    x = xy * trigon760_.cos(h_angle);
    y = xy * trigon760_.sin(h_angle);
    z = distance * trigon760_.sin(v_angle);
  }
} OnePointInfo760;

typedef struct _ClusterItem {
  std::vector<OnePointInfo760*> item;
  std::vector<OnePointInfo760*> outlier;
  float left = 0;
  float rigth = 0;
  float up = 0;
  float down = 0;
} ClusterItem;

class ItIntexRecord760 {
 public:
  long long index_record_ = -1;
  long long it_strong_index_record_ = -1;
  long long it_last_index_record_ = -1;
};

class OnePointInfoManage760 {
 private:
  std::vector<OnePointInfo760> strong_point_data_;
  std::vector<std::vector<long>> strong_point_dataI_index_;

  std::vector<OnePointInfo760> last_point_data_;
  std::vector<std::vector<long>> last_point_data_index_;

  std::vector<ItIntexRecord760> it_index_records_;

  int line_count_ = 0;

 public:
  size_t strong_point_data_size_ = 0;
  size_t last_point_data_size_ = 0;

  const unsigned short max_line_id = 192;

  size_t index_record_size_ = 0;

  OnePointInfoManage760();
  ~OnePointInfoManage760();

  void init(int line_id);
  void insert(unsigned int index, OnePointInfo760& data);
  void insert(unsigned int index, OnePointInfo760&& data);

  OnePointInfo760& getData(int echo_type, unsigned int line_id, unsigned int h_angle);
  OnePointInfo760& getData(unsigned int index);
  long getIndex(int echo_type, unsigned int line_id, unsigned int h_angle);
  OnePointInfo760& getDataIt(unsigned int index);

  void calculatexyz();

  void clear();
  size_t size() const;
};

//-----------------------------------------------------
OnePointInfoManage760::OnePointInfoManage760() {
}

OnePointInfoManage760::~OnePointInfoManage760() {
}

void OnePointInfoManage760::init(int line_id) {
  // auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < max_line_id; i++) {
    std::vector<long> v, v2;
    strong_point_dataI_index_.push_back(v);
    last_point_data_index_.push_back(v2);
    for (size_t j = 0; j < MAX_COL_NUM_760; j++) {
      strong_point_dataI_index_[i].push_back(-1);
      last_point_data_index_[i].push_back(-1);
    }
  }
  strong_point_data_size_ = 0;
  last_point_data_size_ = 0;
  line_count_ = line_id;
  index_record_size_ = 0;
}

void OnePointInfoManage760::clear() {
  strong_point_data_.clear();
  last_point_data_.clear();
  strong_point_dataI_index_.clear();
  last_point_data_index_.clear();
  it_index_records_.clear();
  for (size_t i = 0; i < max_line_id; i++) {
    std::vector<long> v(MAX_COL_NUM_760, -1), v2(MAX_COL_NUM_760, -1);

    strong_point_dataI_index_.push_back(v);
    last_point_data_index_.push_back(v2);
  }
  strong_point_data_size_ = 0;
  last_point_data_size_ = 0;
  index_record_size_ = 0;
}

inline void OnePointInfoManage760::insert(unsigned int index, OnePointInfo760& data) {
  // Q_ASSERT(PointData);
  if (data.echo_type == 1) {
    if (strong_point_dataI_index_[data.line_id - 1][data.hor_angle_index] == -1) {
      strong_point_dataI_index_[data.line_id - 1][data.hor_angle_index] = strong_point_data_size_;
      strong_point_data_.push_back(data);
      ItIntexRecord760 rtem;
      rtem.index_record_ = index_record_size_;
      rtem.it_strong_index_record_ = strong_point_data_.size() - 1;
      it_index_records_.push_back(rtem);
      strong_point_data_size_++;
      index_record_size_++;
    } else {
      strong_point_data_[strong_point_dataI_index_[data.line_id - 1][data.hor_angle_index]] = data;
    }
  } else {
    if (last_point_data_index_[data.line_id - 1][data.hor_angle_index] == -1) {
      last_point_data_index_[data.line_id - 1][data.hor_angle_index] = last_point_data_size_;
      last_point_data_.push_back(data);
      ItIntexRecord760 rtem;
      rtem.index_record_ = index_record_size_;
      rtem.it_last_index_record_ = last_point_data_.size() - 1;
      it_index_records_.push_back(rtem);
      last_point_data_size_++;
      index_record_size_++;
    } else {
      last_point_data_[last_point_data_index_[data.line_id - 1][data.hor_angle_index]] = data;
    }
  }
}

inline void OnePointInfoManage760::insert(unsigned int index, OnePointInfo760&& data) {
  if (data.echo_type == 1) {
    if (strong_point_dataI_index_[data.line_id - 1][data.hor_angle_index] == -1) {
      strong_point_dataI_index_[data.line_id - 1][data.hor_angle_index] = strong_point_data_size_;
      strong_point_data_.push_back(data);
      ItIntexRecord760 rtem;
      rtem.index_record_ = index_record_size_;
      rtem.it_strong_index_record_ = strong_point_data_.size() - 1;
      it_index_records_.push_back(rtem);
      strong_point_data_size_++;
      index_record_size_++;
    } else {
      strong_point_data_[strong_point_dataI_index_[data.line_id - 1][data.hor_angle_index]] = data;
    }
  } else {
    if (last_point_data_index_[data.line_id - 1][data.hor_angle_index] == -1) {
      last_point_data_index_[data.line_id - 1][data.hor_angle_index] = last_point_data_size_;
      last_point_data_.push_back(data);
      ItIntexRecord760 rtem;
      rtem.index_record_ = index_record_size_;
      rtem.it_last_index_record_ = last_point_data_.size() - 1;
      it_index_records_.push_back(rtem);
      last_point_data_size_++;
      index_record_size_++;
    } else {
      last_point_data_[last_point_data_index_[data.line_id - 1][data.hor_angle_index]] = data;
    }
  }
}

OnePointInfo760 res_760;

inline OnePointInfo760& OnePointInfoManage760::getData(int echo_type, unsigned int line_id, unsigned int hor_angle_index) {
  assert(echo_type < 3);
  assert(line_id - 1 < line_count_);
  assert(hor_angle_index <= MAX_COL_NUM_760);
  if (echo_type == 1) {
    if (strong_point_dataI_index_[line_id - 1][hor_angle_index] >= 0) {
      return strong_point_data_[strong_point_dataI_index_[line_id - 1][hor_angle_index]];
    }
  } else {
    if (last_point_data_index_[line_id - 1][hor_angle_index] >= 0) {
      return last_point_data_[last_point_data_index_[line_id - 1][hor_angle_index]];
    }
  }
  return res_760;
}

inline OnePointInfo760& OnePointInfoManage760::getData(unsigned int index) {
  int echo_type = (index >= max_line_id * MAX_COL_NUM_760) ? 2 : 1;
  int line_id = (echo_type == 1) ? (index / MAX_COL_NUM_760) : ((index - (max_line_id * MAX_COL_NUM_760)) / MAX_COL_NUM_760);
  int hor_angle_index = (echo_type == 1) ? (index % MAX_COL_NUM_760) : ((index - (max_line_id * MAX_COL_NUM_760)) % MAX_COL_NUM_760);

  if (echo_type == 1) {
    if (strong_point_dataI_index_[line_id][hor_angle_index] >= 0) {
      return strong_point_data_[strong_point_dataI_index_[line_id][hor_angle_index]];
    }
  } else {
    if (last_point_data_index_[line_id][hor_angle_index] >= 0) {
      return last_point_data_[last_point_data_index_[line_id][hor_angle_index]];
    }
  }
  return res_760;
}

long OnePointInfoManage760::getIndex(int echo_type, unsigned int line_id, unsigned int hor_angle_index) {
  assert(echo_type < 3);
  assert(line_id <= line_count_);
  assert(hor_angle_index <= MAX_COL_NUM_760);
  if (echo_type == 1) {
    return strong_point_dataI_index_[line_id - 1][hor_angle_index];
  } else {
    return last_point_data_index_[line_id - 1][hor_angle_index];
  }
  return -1;
}

OnePointInfo760& OnePointInfoManage760::getDataIt(unsigned int index) {
  assert(index < index_record_size_);

  ItIntexRecord760 rtem = it_index_records_[index];

  if (rtem.it_strong_index_record_ != -1) {
    return strong_point_data_[rtem.it_strong_index_record_];
  } else if (rtem.it_last_index_record_ != -1) {
    return last_point_data_[rtem.it_last_index_record_];
  }

  return res_760;
}

size_t OnePointInfoManage760::size() const {
  return index_record_size_;
}
