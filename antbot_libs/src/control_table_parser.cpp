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

#include <string>
#include <unordered_map>

#include "antbot_libs/control_table_parser.hpp"

namespace antbot
{
namespace libs
{
static rclcpp::Logger logger = rclcpp::get_logger("ControlTableParser");

ControlTableParser::ControlTableParser()
{
}

ControlTableParser::~ControlTableParser()
{
}

bool ControlTableParser::load_xml_file(const char * file)
{
  if (tinyxml2::XMLError::XML_SUCCESS == doc_.LoadFile(file)) {
    RCLCPP_INFO(logger, "Loaded XML file: %s", file);
  } else {
    RCLCPP_ERROR(logger, "Failed to open XML file: %s", file);
    return false;
  }

  return true;
}

uint16_t ControlTableParser::parse_min_address()
{
  tinyxml2::XMLElement * device_element = doc_.FirstChildElement("Device");
  if (device_element == nullptr) {
    RCLCPP_WARN(logger, "Failed to find <Device> element, using default MinAddress: 0");
    return 0;
  }

  int val = 0;
  if (device_element->QueryIntAttribute("MinAddress", &val) == tinyxml2::XML_SUCCESS) {
    RCLCPP_INFO(logger, "MinAddress: %d", val);
    return static_cast<uint16_t>(val);
  }
  RCLCPP_WARN(logger, "MinAddress attribute not found, using default: 0");
  return 0;
}

uint16_t ControlTableParser::parse_max_address()
{
  tinyxml2::XMLElement * device_element = doc_.FirstChildElement("Device");
  if (device_element == nullptr) {
    RCLCPP_WARN(logger, "Failed to find <Device> element, using default MaxAddress: 255");
    return 255;
  }

  int val = 255;
  if (device_element->QueryIntAttribute("MaxAddress", &val) == tinyxml2::XML_SUCCESS) {
    RCLCPP_INFO(logger, "MaxAddress: %d", val);
    return static_cast<uint16_t>(val);
  }
  RCLCPP_WARN(logger, "MaxAddress attribute not found, using default: 255");
  return val;
}

bool ControlTableParser::parse_items()
{
  tinyxml2::XMLElement * control_items_element;

  tinyxml2::XMLElement * device_el = doc_.FirstChildElement("Device");
  if (device_el == nullptr) {return false;}

  tinyxml2::XMLElement * items_el = device_el->FirstChildElement("ControlItems");
  if (items_el == nullptr) {return false;}

  control_items_element = items_el->FirstChildElement("Item");
  if (control_items_element == nullptr) {
    RCLCPP_WARN(logger, "Can't find <Item> elements");
    return false;
  }

  while (control_items_element != nullptr) {
    try {
      const char * name = control_items_element->Attribute("Name");
      if (name == nullptr) {
        RCLCPP_ERROR(logger, "Item missing 'Name' attribute, skipping");
        control_items_element = control_items_element->NextSiblingElement("Item");
        continue;
      }
      int64_t address = 0, length = 0, rw = 0;

      control_items_element->QueryInt64Attribute("Address", &address);
      control_items_element->QueryInt64Attribute("Length", &length);
      control_items_element->QueryInt64Attribute("RW", &rw);

      RCLCPP_DEBUG(
        logger,
        "name : %s, address : %ld, length : %ld, rw : %ld",
        name,
        address,
        length,
        rw);

      auto item = this->get_control_item(address, length, rw);
      control_table_[name] = item;

      control_items_element = control_items_element->NextSiblingElement("Item");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "%s", e.what());
      return false;
    }
  }

  return true;
}

bool ControlTableParser::parse_control_table()
{
  return this->parse_items();
}

ControlTableParser::ControlItem ControlTableParser::get_control_item(
  const uint16_t & address,
  const uint16_t & length,
  const uint8_t & rw)
{
  ControlItem item = {address, length, rw};
  return item;
}

std::unordered_map<std::string, ControlTableParser::ControlItem>
ControlTableParser::get_control_table()
{
  return control_table_;
}
}  // namespace libs
}  // namespace antbot
