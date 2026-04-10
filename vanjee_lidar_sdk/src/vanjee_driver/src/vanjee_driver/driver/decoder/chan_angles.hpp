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

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <vanjee_driver/common/wj_common.hpp>
#include <vanjee_driver/common/wj_log.hpp>
using namespace std;

namespace vanjee {
namespace lidar {
enum RotateDirection { anticlockwise = 0, clockwise };
#pragma pack(push, 1)
typedef struct {
  uint8_t sign;
  uint16_t value;
} WJSCalibrationAngle;
#pragma pack(pop)
class ChanAngles {
 private:
  uint16_t chan_num_;
  std::vector<int32_t> eccentricity_angles_;
  std::vector<int32_t> vert_angles_;
  std::vector<int32_t> horiz_angles_;
  std::vector<vector<int32_t>> horiz_vert_angles_;
  std::vector<uint16_t> user_chans_;

 public:
  ChanAngles(uint16_t chan_num) : chan_num_(chan_num) {
    vert_angles_.resize(chan_num_);
    horiz_angles_.resize(chan_num_);
    horiz_vert_angles_.resize(chan_num_ * 3);
    for (size_t i = 0; i < horiz_vert_angles_.size(); i++) {
      horiz_vert_angles_[i].resize(1200);
    }
    user_chans_.resize(chan_num_);
  }

  /// <summary>
  /// String splitting
  /// </summary>
  /// <param name="strSur">A string that needs to be split</param>
  /// <param name="cConChar">Split characters</param>
  /// <returns>Return the split substrings</returns>
  std::vector<std::string> vStrSplit(std::string strSur, char cConChar) {
    std::vector<std::string> vStrVec;
    std::string::size_type pos1, pos2;
    pos1 = 0;
    pos2 = strSur.find(cConChar, 0);

    while (std::string::npos != pos2) {
      vStrVec.push_back(strSur.substr(pos1, pos2 - pos1));
      pos1 = pos2 + 1;
      pos2 = strSur.find(cConChar, pos1);
    }
    vStrVec.push_back(strSur.substr(pos1));
    return vStrVec;
  }

  int loadFromFile(const std::string &angle_path) {
    std::vector<int32_t> vert_angles;
    std::vector<int32_t> horiz_angles;
    int ret = loadFromFile(angle_path, chan_num_, vert_angles, horiz_angles);
    if (ret < 0)
      return ret;

    if (vert_angles.size() != chan_num_) {
      return -1;
    }

    vert_angles_.swap(vert_angles);
    horiz_angles_.swap(horiz_angles);
    genUserChan(vert_angles_, user_chans_);
    return 0;
  }

  int loadFromFile(const std::string &vangle_path, const std::string &hangle_path) {
    std::vector<int32_t> vert_angles;
    std::vector<::vector<int32_t>> horiz_vert_angles;
    int ret = loadFromFile(vangle_path, hangle_path, chan_num_, vert_angles, horiz_vert_angles);
    if (ret < 0)
      return ret;

    if (vert_angles.size() != chan_num_) {
      return -1;
    }

    vert_angles_.swap(vert_angles);
    horiz_vert_angles_.swap(horiz_vert_angles);
    genUserChan(vert_angles_, user_chans_);
    return 0;
  }

  int loadFromFile(const std::string &angle_path, size_t size) {
    std::vector<int32_t> eccentricity_angles;
    int ret = loadFromFile(angle_path, size, eccentricity_angles);
    if (ret < 0)
      return ret;

    if (eccentricity_angles.size() != size) {
      return -1;
    }
    eccentricity_angles_.resize(size);
    eccentricity_angles_.swap(eccentricity_angles);
    return 0;
  }

  int loadFromLiDAR(const std::string &angle_path, uint16_t num, std::vector<double> vert_angles, std::vector<double> horiz_angles) {
    for (uint16_t i = 0; i < num; i++) {
      vert_angles_[i] = (static_cast<int32_t>(vert_angles[i] * 1000));
      horiz_angles_[i] = (static_cast<int32_t>(horiz_angles[i] * 1000));
    }

    std::ofstream fd(angle_path.c_str(), std::ios::out);
    if (!fd.is_open()) {
      WJ_WARNING << "fail to open angle file:" << angle_path << WJ_REND;
      return -1;
    }

    std::string line;
    for (uint16_t i = 0; i < num; i++) {
      if (horiz_angles[0] != 0) {
        fd << std::to_string(vert_angles[i]) << "," << std::to_string(horiz_angles[i]) << "\n";
      } else {
        fd << std::to_string(vert_angles[i]) << "," << std::to_string(horiz_angles_[i]) << "\n";
      }
    }

    fd.close();
    return 0;
  }

  int loadFromLiDAR(const std::string &angle_path, uint16_t num, std::vector<int32_t> eccentricity_angles) {
    if (eccentricity_angles.size() == 0)
      return -1;
    eccentricity_angles_.clear();
    for (uint16_t i = 0; i < num; i++) {
      eccentricity_angles_.push_back(static_cast<int32_t>(eccentricity_angles[i]));
    }

    std::ofstream fd(angle_path.c_str(), std::ios::out);
    if (!fd.is_open()) {
      WJ_WARNING << "fail to open angle file:" << angle_path << WJ_REND;
      return -1;
    }

    std::string line;
    fd << std::to_string(eccentricity_angles[0]);
    for (uint16_t i = 1; i < num; i++) {
      fd << "," << std::to_string(eccentricity_angles[i]);
    }
    fd << "\n";

    fd.close();
    return 0;
  }

  /// @brief Obtain the user channel number from the given radar internal
  /// channel number
  uint16_t toUserChan(uint16_t chan) {
    return user_chans_[chan];
  }

  /// @brief Correct the horizontal angle given by the parameters
  int32_t eccentricityAdjust(int32_t horiz_index, int32_t resolution) {
    return eccentricity_angles_[horiz_index] * resolution;
  }

  int32_t horizAdjust(uint16_t chan, int32_t horiz, RotateDirection rotate_direction = RotateDirection::anticlockwise) {
    if (rotate_direction == RotateDirection::anticlockwise)
      return (horiz + horiz_angles_[chan]);
    else
      return (horiz - horiz_angles_[chan]);
  }
  int32_t vertAdjust(uint16_t chan) {
    return vert_angles_[chan];
  }

  int32_t horiz_vertAdjust(uint16_t chan, int32_t horiz) {
    return (horiz_vert_angles_[chan][horiz]);
  }

 private:
  /// @brief Calculate the user channel number array in ascending order based on
  /// the angle values in the member variable 'vert_angles []'
  void genUserChan(const std::vector<int32_t> &vert_angles, std::vector<uint16_t> &user_chans) {
    user_chans.resize(vert_angles.size());

    for (size_t i = 0; i < vert_angles.size(); i++) {
      int32_t angle = vert_angles[i];
      uint16_t chan = 0;

      for (size_t j = 0; j < vert_angles.size(); j++) {
        if (vert_angles[j] < angle) {
          chan++;
        }
      }

      user_chans[i] = chan;
    }
  }

  int loadFromFile(const std::string &angle_path, size_t size, std::vector<int32_t> &vert_angles, std::vector<int32_t> &horiz_angles) {
    vert_angles.clear();
    horiz_angles.clear();

    std::ifstream fd(angle_path.c_str(), std::ios::in);
    if (!fd.is_open()) {
      WJ_WARNING << "fail to open vangle file:" << angle_path << WJ_REND;
      return -1;
    }

    std::string line;
    for (size_t i = 0; i < size; i++) {
      try {
        if (!std::getline(fd, line))
          return -1;

        float vert = std::stof(line);

        float horiz = 0;
        size_t pos_comma = line.find_first_of(',');
        if (pos_comma != std::string::npos) {
          horiz = std::stof(line.substr(pos_comma + 1));
        }

        vert_angles.emplace_back(static_cast<int32_t>(vert * 1000));
        horiz_angles.emplace_back(static_cast<int32_t>(horiz * 1000));
      } catch (...) {
        WJ_ERROR << "The format of angle config file " << angle_path << " is wrong. Please check (e.g. indentation)." << WJ_REND;
      }
    }
    fd.close();
    return 0;
  }

  int loadFromFile(const std::string &vangle_path, const std::string &hangle_path, size_t size, std::vector<int32_t> &vert_angles,
                   std::vector<std::vector<int32_t>> &horiz_vert_angles) {
    vert_angles.clear();
    horiz_vert_angles.clear();

    horiz_vert_angles.resize(size * 3);
    for (size_t i = 0; i < horiz_vert_angles.size(); i++) {
      horiz_vert_angles[i].resize(1200);
    }

    std::ifstream fd_v(vangle_path.c_str(), std::ios::in);
    std::ifstream fd_h(hangle_path.c_str(), std::ios::in);

    if (!fd_v.is_open()) {
      WJ_WARNING << "fail to open vangle file:" << vangle_path << WJ_REND;
      return -1;
    }
    if (!fd_h.is_open()) {
      WJ_WARNING << "fail to open hangle file:" << hangle_path << WJ_REND;
      return -1;
    }

    std::string line;
    for (size_t i = 0; i < size; i++) {
      if (!std::getline(fd_v, line))
        return -1;

      float vert = std::stof(line);
      vert_angles.emplace_back(static_cast<int32_t>(vert * 1000));
    }

    int rowidx = 0;
    std::string h_line;
    // std::vector<int32_t> h_p_pointNum;
    while (std::getline(fd_h, h_line)) {
      std::stringstream sinFile(h_line);
      std::vector<std::string> LineData = ChanAngles::vStrSplit(sinFile.str(), ',');
      int ch = ((rowidx + 1) % 3) == 0 ? (rowidx + 1) / 3 : (rowidx + 1) / 3 + 1;
      int hvidx = 1199;

      for (size_t i = 0; i < 1200; i++) {
        double v = 0;
        double VAngle = 0;
        switch ((rowidx + 1) % 3) {
          case 0:
            v = std::stod(LineData[hvidx]) * 1000;
            VAngle = vert_angles[ch - 1] + v;
            horiz_vert_angles[rowidx][i] = VAngle;
            break;

          case 1:
            v = std::stod(LineData[hvidx]) * 1000;
            VAngle = vert_angles[ch - 1] + v;
            horiz_vert_angles[rowidx + 1][i] = VAngle;
            break;

          case 2:
            v = std::stod(LineData[hvidx]) * 1000;
            VAngle = vert_angles[ch - 1] + v;
            horiz_vert_angles[rowidx - 1][i] = VAngle;
            break;
            ;
        }
        hvidx--;
      }

      rowidx++;
    }
    fd_v.close();
    fd_h.close();
    return 0;
  }

  int loadFromFile(const std::string &angle_path, size_t size, std::vector<int32_t> &eccentricity_angles) {
    eccentricity_angles.clear();

    std::ifstream fd(angle_path.c_str(), std::ios::in);
    if (!fd.is_open()) {
      WJ_WARNING << "fail to open vangle file:" << angle_path << WJ_REND;
      return -1;
    }
    std::string item;
    while (std::getline(fd, item, ',')) {
      item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
      if (!item.empty()) {
        try {
          eccentricity_angles.push_back(static_cast<int32_t>(std::stoi(item)));
        } catch (const std::invalid_argument &e) {
          std::cerr << "Invalid argument: " << item << " is not a valid integer." << std::endl;
        } catch (const std::out_of_range &e) {
          std::cerr << "Out of range: " << item << " is out of int32_t range." << std::endl;
        }
      }
    }
    fd.close();
    if (eccentricity_angles.size() != size)
      return -1;
    return 0;
  }

  bool angleCheck(int32_t v) {
    return ((-9000 <= v) && (v < 9000));
  }
};

}  // namespace lidar
}  // namespace vanjee
