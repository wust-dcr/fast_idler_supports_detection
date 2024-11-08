// Copyright 2024 Jakub Delicat
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

#include <fast_idler_supports_detection/utils.hpp>

void print_diffrence(const std::string &logger_name, CloudPtr cloud1,
                     CloudPtr cloud2) {
  const auto removed_points_count =
      cloud1->points.size() - cloud2->points.size();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name),
                     "Got: " << cloud1->points.size() << " points.");
  RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name),
                     "Removed: " << removed_points_count << " points.");
}

CloudPtr remove_intensity_from_cloud(CloudIPtr cloud) {
  CloudPtr new_cloud(new Cloud);
  for (const auto &point : cloud->points) {
    Point new_point;
    new_point.x = point.x;
    new_point.y = point.y;
    new_point.z = point.z;
    new_cloud->points.push_back(new_point);
  }

  new_cloud->width = new_cloud->size();
  new_cloud->height = 1;
  return new_cloud;
}

bool is_point_inside_box(const Point &point, const BoundingBox &box) {
  return (point.x >= box.center.position.x - box.size.x / 2 &&
          point.x <= box.center.position.x + box.size.x / 2 &&
          point.y >= box.center.position.y - box.size.y / 2 &&
          point.y <= box.center.position.y + box.size.y / 2 &&
          point.z >= box.center.position.z - box.size.z / 2 &&
          point.z <= box.center.position.z + box.size.z / 2);
}

void print_cloud(rclcpp::Node *node, CloudPtr cloud) {
  std::ostringstream str;
  for (const auto &point : cloud->points) {
    str << point;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), str.str());
}

void print_cloud(rclcpp::Node *node, CloudIPtr cloud) {
  std::ostringstream str;
  for (const auto &point : cloud->points) {
    str << point;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), str.str());
}

void print_cloud(rclcpp::Node *node, CloudIRPtr cloud) {
  std::ostringstream str;
  for (const auto &point : cloud->points) {
    str << point;
  }
  RCLCPP_INFO_STREAM(node->get_logger(), str.str());
}
