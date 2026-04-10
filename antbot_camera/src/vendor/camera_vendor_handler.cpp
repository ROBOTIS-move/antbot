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
// Authors: Jaehun Park

#include "antbot_camera/vendor/camera_vendor_handler.hpp"
#include "antbot_camera/vendor/novitec_camera_handler.hpp"
#include "antbot_camera/vendor/generic_camera_handler.hpp"

namespace antbot_camera
{
namespace vendor
{

std::shared_ptr<CameraVendorHandler> create_vendor_handler(
  const std::string & vendor_name,
  const std::string & port_root)
{
  if (vendor_name == "novitec") {
    return std::make_shared<NovitecCameraHandler>(port_root);
  } else {
    return std::make_shared<GenericCameraHandler>(port_root);
  }
}

}  // namespace vendor
}  // namespace antbot_camera
