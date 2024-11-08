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

#pragma once

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fast_idler_supports_detection/point_types.hpp>

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;
using CloudPtrs = std::vector<CloudPtr>;

using PointIR = pcl::PointXYZIR;
using CloudIR = pcl::PointCloud<PointIR>;
using CloudIRPtr = CloudIR::Ptr;
using CloudIRPtrs = std::vector<CloudIRPtr>;

using PointIRL = pcl::PointXYZIRL;
using CloudIRL = pcl::PointCloud<PointIRL>;
using CloudIRLPtr = CloudIRL::Ptr;
using CloudIRLPtrs = std::vector<CloudIRLPtr>;

using PointI = pcl::PointXYZI;
using CloudI = pcl::PointCloud<PointI>;
using CloudIPtr = CloudI::Ptr;
using CloudIPtrs = std::vector<CloudIPtr>;

using Normal = pcl::PointCloud<pcl::Normal>;
using NormalPtr = Normal::Ptr;
using NormalPtrs = std::vector<NormalPtr>;
using CloudNormal = pcl::PointCloud<Normal>;
using CloudNormalPtr = CloudNormal::Ptr;

using rclcppCloud = sensor_msgs::msg::PointCloud2;
using rclcppCloudSharedPtr = rclcppCloud::SharedPtr;

using MarkersPtr = std::shared_ptr<visualization_msgs::msg::MarkerArray>;
using BoundingBox = vision_msgs::msg::BoundingBox3D;
using BoundingBoxArrayPtr =
    std::shared_ptr<vision_msgs::msg::BoundingBox3DArray>;
using Detection3D = vision_msgs::msg::Detection3D;
using Detection3DArrayPtr = std::shared_ptr<vision_msgs::msg::Detection3DArray>;

void print_difference(const std::string &logger_name, CloudPtr cloud1,
                      CloudPtr cloud2);

CloudPtr remove_intensity_from_cloud(CloudIPtr cloud);

bool is_point_inside_box(const Point &point, const BoundingBox &box);

// TODO: template this:
void print_cloud(rclcpp::Node *node, CloudPtr cloud);
void print_cloud(rclcpp::Node *node, CloudIPtr cloud);
void print_cloud(rclcpp::Node *node, CloudIRPtr cloud);

struct Histogram {
  std::vector<std::vector<std::size_t>> data;
  std::size_t image_width;
  std::size_t image_height;
  double width;
  double length;
  double height;
  double resolution;
  Point origin;

  std::vector<std::size_t> &operator[](std::size_t index) {
    return data[index];
  }
  std::vector<std::size_t> operator[](std::size_t index) const {
    return data[index];
  }
};
