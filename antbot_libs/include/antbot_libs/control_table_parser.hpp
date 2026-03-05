// Copyright 2026 ROBOTIS AI CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Darby Lim, Daun Jeong

#ifndef ANTBOT_LIBS__CONTROL_TABLE_PARSER_HPP_
#define ANTBOT_LIBS__CONTROL_TABLE_PARSER_HPP_

#include <tinyxml2.h>
#include <string>
#include <unordered_map>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace antbot
{
namespace libs
{
class ControlTableParser
{
public:
  ControlTableParser();
  virtual ~ControlTableParser();

  bool load_xml_file(const char * file);

  uint16_t parse_min_address();
  uint16_t parse_max_address();

  bool parse_control_table();

  struct ControlItem
  {
    uint16_t address;
    uint16_t length;
    uint8_t rw;
  };

  std::unordered_map<std::string, ControlItem> get_control_table();

  ControlItem get_control_item(
    const uint16_t & address,
    const uint16_t & length,
    const uint8_t & rw);

private:
  bool parse_items();

  tinyxml2::XMLDocument doc_;
  std::unordered_map<std::string, ControlItem> control_table_;
};
}  // namespace libs
}  // namespace antbot
#endif  // ANTBOT_LIBS__CONTROL_TABLE_PARSER_HPP_
