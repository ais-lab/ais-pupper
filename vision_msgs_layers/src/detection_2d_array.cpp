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

#include <QPainter>
#include "vision_msgs_layers/detection_2d_array.hpp"

namespace vision_msgs_layers
{

void Detection2DArray::overlay(
  QPainter & painter,
  const vision_msgs::msg::Detection2DArray & msg)
{
  painter.save();

  QPen pen = painter.pen();
  pen.setWidth(2);
  painter.setPen(pen);

  // Draw Bounding Boxes and Confidence for each detection
  for (const auto & detection : msg.detections) {
    const auto & bbox = detection.bbox;
    painter.save();
    painter.translate(bbox.center.position.x, bbox.center.position.y);
    painter.rotate(-bbox.center.theta * 180.0 / 3.141592654);
    painter.drawRect(-bbox.size_x / 2, -bbox.size_y / 2, bbox.size_x, bbox.size_y);
    painter.restore();
  }

  painter.restore();
}

}  // namespace vision_msgs_layers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(vision_msgs_layers::Detection2DArray, rqt_image_overlay_layer::PluginInterface)
