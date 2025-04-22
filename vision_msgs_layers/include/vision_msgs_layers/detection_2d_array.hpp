// Copyright 2025 Charlene Vernant
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

#ifndef VISION_MSGS_LAYERS__DETECTION_2D_ARRAY_HPP_
#define VISION_MSGS_LAYERS__DETECTION_2D_ARRAY_HPP_

#include "vision_msgs_layers/visibility_control.h"
#include "rqt_image_overlay_layer/plugin.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace vision_msgs_layers
{

class Detection2DArray : public rqt_image_overlay_layer::Plugin<vision_msgs::msg::Detection2DArray>
{
public:
  void overlay(
    QPainter & painter,
    const vision_msgs::msg::Detection2DArray & msg) override;
};

}  // namespace vision_msgs_layers

#endif  // VISION_MSGS_LAYERS__DETECTION_2D_ARRAY_HPP_
