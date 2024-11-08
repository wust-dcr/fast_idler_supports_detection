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
#define PCL_NO_PRECOMPILE
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fast_idler_supports_detection/point_types.hpp>

namespace pcl_utils {

template <typename PointT>
void save_cloud(const std::string &name,
                typename pcl::PointCloud<PointT>::Ptr cloud) {
  pcl::PCDWriter writer;

  std::string path_to_pcds = ament_index_cpp::get_package_share_directory(
      "fast_idler_supports_detection");
  path_to_pcds += std::string("/test_data/");
  auto path = path_to_pcds + std::string(name);
  writer.write<PointT>(path, *cloud);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
remove_far_points_from_ros2bag_converter_bug(
    typename pcl::PointCloud<PointT>::Ptr &cloud, double max_distance) {
  auto new_cloud = typename pcl::PointCloud<PointT>::Ptr(
      new typename pcl::PointCloud<PointT>);
  for (const auto &point : cloud->points) {
    const auto distance = pcl::euclideanDistance(point, Point{0, 0, 0});
    if (distance < max_distance) {
      new_cloud->points.push_back(point);
    }
  }
  new_cloud->height = new_cloud->points.size();
  new_cloud->width = 1;
  return new_cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
convert_point_cloud2_to_cloud_ptr(rclcppCloudSharedPtr pc2) {
  pcl::PCLPointCloud2 temp_cloud;
  typename pcl::PointCloud<PointT>::Ptr cloud(new
                                              typename pcl::PointCloud<PointT>);
  pcl_conversions::toPCL(*pc2, temp_cloud);
  pcl::fromPCLPointCloud2(temp_cloud, *cloud);
  return cloud;
}

template <typename PointT>
rclcppCloud
convert_cloud_ptr_to_point_cloud2(typename pcl::PointCloud<PointT>::Ptr cloud,
                                  const std::string &frame_name,
                                  rclcpp::Node *node) {
  sensor_msgs::msg::PointCloud2 point_cloud;
  pcl::PCLPointCloud2 point_cloud2;
  pcl::toPCLPointCloud2(*cloud, point_cloud2);
  pcl_conversions::fromPCL(point_cloud2, point_cloud);

  point_cloud.header.frame_id = frame_name;
  point_cloud.header.stamp = node->get_clock()->now();
  point_cloud.is_dense = true;
  return point_cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
rotate(typename pcl::PointCloud<PointT>::Ptr cloud, double roll, double pitch,
       double yaw) {
  if (not cloud) {
    return nullptr;
  }
  typename pcl::PointCloud<PointT>::Ptr transformed_cloud(
      new pcl::PointCloud<PointT>);
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  transform_2.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
  transform_2.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
  transform_2.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);
  return transformed_cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
translate(typename pcl::PointCloud<PointT>::Ptr cloud, double x, double y,
          double z) {
  typename pcl::PointCloud<PointT>::Ptr transformed_cloud(
      new pcl::PointCloud<PointT>);
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << x, y, z;
  pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);
  return transformed_cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
merge_clouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds) {
  typename pcl::PointCloud<PointT>::Ptr merged_clouds(
      new typename pcl::PointCloud<PointT>);
  for (const auto &cloud : clouds) {
    if (cloud->size()) {
      *merged_clouds += *cloud;
    }
  }
  merged_clouds->width = merged_clouds->size();
  merged_clouds->height = 1;
  return merged_clouds;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr merge_clouds_and_remove_simillar_points(
    const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
    double eps) {
  typename pcl::PointCloud<PointT>::Ptr new_cloud(
      new typename pcl::PointCloud<PointT>);
  for (const auto &cloud : clouds) {
    *new_cloud += *cloud;
  }
  for (std::size_t i = 0; i < new_cloud->size(); ++i) {
    for (std::size_t j = 0; j < new_cloud->size(); ++j) {
      if (i == j)
        continue;
      const double distance =
          pcl::euclideanDistance(new_cloud->points[i], new_cloud->points[j]);
      if (distance < eps) {
        new_cloud->points.erase(new_cloud->points.begin() + j);
      }
    }
  }
  new_cloud->height = new_cloud->points.size();
  new_cloud->width = 1;
  return new_cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr remove_simillar_points(
    const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
    double eps) {
  typename pcl::PointCloud<PointT>::Ptr new_cloud(
      new typename pcl::PointCloud<PointT>);
  for (const auto &point : clouds[0]->points) {
    bool found_similar = false;

    for (std::size_t j = 1; j < clouds.size(); ++j) {
      for (const auto &point_compare : clouds[j]->points) {
        const double distance = pcl::euclideanDistance(point, point_compare);
        if (distance <= eps) {
          found_similar = true;
        }
      }
    }
    if (not found_similar) {
      new_cloud->points.push_back(point);
    }
  }

  new_cloud->height = new_cloud->points.size();
  new_cloud->width = 1;
  return new_cloud;
}
} // namespace pcl_utils
