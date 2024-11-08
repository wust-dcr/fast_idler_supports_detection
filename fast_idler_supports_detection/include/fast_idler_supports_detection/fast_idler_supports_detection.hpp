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

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include <pcl/common/distances.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fast_idler_supports_detection/cluster_extraction.hpp>
#include <fast_idler_supports_detection/pcl_utils.hpp>
#include <fast_idler_supports_detection/utils.hpp>

using namespace std::chrono_literals;

class FastIdlerSupportsDetection : public rclcpp::Node {
public:
  FastIdlerSupportsDetection();

private:
  void declare_parameters();
  void get_parameters();

  std::string pointcloud_topic_name;
  double ground_level_height;
  double tunnel_width;
  double tunnel_height;
  double tunnel_length;

  double roll;
  double pitch;
  double yaw;

  double x;
  double y;
  double z;
  double forward_resolution;
  std::size_t forward_histogram_min;
  std::size_t forward_histogram_max;
  std::size_t forward_column_density_threshold;

  double ground_resolution;
  std::size_t ground_histogram_min;
  std::size_t ground_histogram_max;
  double ground_histogram_a;

  std::size_t max_detected_legs = 0;
  std::size_t counter = 0;

  std::size_t slope;
  std::size_t range;

  void create_rclcpp_instances();
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<rclcppCloud>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr distance_filtered_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr without_ground_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr transformed_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_conveyors_candidates_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_conveyors_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr forward_hist_filtered_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr top_hist_filtered_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr merged_density_clouds_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr clustered_supports_candidates_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr
      clustered_supports_candidates_velodyne_pub_;
  rclcpp::Publisher<rclcppCloud>::SharedPtr
      clustered_supports_candidates_base_link_pub_;

  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
      conveyors_detection_3d_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
      supports_detection_3d_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr
      supports_detection_base_link_3d_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      forward_density_histogram_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      forward_density_clustered_histogram_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      ground_density_histogram_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      ground_density_clustered_histogram_pub_;

  ClusterExtraction supports_candidates_clusteler;
  ClusterExtraction conveyor_candidates_clusteler;

  CloudPtr remove_points_beyond_tunnel(CloudPtr cloud);
  CloudIRLPtr filter_with_density_on_x_image(CloudIRLPtr cloud,
                                             const Histogram &histogram);
  CloudIRLPtr filter_with_density_on_z_image(CloudIRLPtr cloud,
                                             const Histogram &histogram);
  std::pair<CloudIRPtr, CloudIRPtr> filter_ground_and_get_normal_and_height(
      CloudIRPtr cloud, int sac_model, int iterations, double radius,
      Eigen::Vector3d &normal, double &ground_height, double eps = 0);

  CloudIRPtr align_to_normal(CloudIRPtr cloud, const Eigen::Vector3d &normal,
                             double ground_height, Eigen::Vector3d &rpy);

  CloudPtr get_points_from_bounding_boxes(CloudPtr cloud,
                                          BoundingBoxArrayPtr boxes);

  Histogram create_top_histogram(CloudIRLPtr cloud, double resolution);
  Histogram create_histogram(CloudIRLPtr cloud, double resolution);
  Histogram remove_low_density_columns(const Histogram &histogram,
                                       std::size_t threshold);
  Histogram threshold_histogram(const Histogram &histogram, std::size_t min,
                                std::size_t max);
  Histogram segment_local_peeks(const Histogram &histogram, std::size_t slope,
                                std::size_t range = 1);

  sensor_msgs::msg::Image
  create_image_from_histogram(const Histogram &histogram);

  BoundingBoxArrayPtr
  make_bounding_boxes_from_pointclouds(const CloudIRLPtrs &clustered_clouds,
                                       const std::string &frame_name);
  vision_msgs::msg::ObjectHypothesisWithPose
  score_conveyor(const vision_msgs::msg::BoundingBox3D bbox);
  Detection3DArrayPtr detect_conveyors(const CloudIRLPtrs &clustered_clouds,
                                       const std::string &frame_name);
  Detection3DArrayPtr detect_supports(const CloudIRLPtrs &clustered_clouds,
                                      const std::string &frame_name);

  void save_data_to_yaml(const sensor_msgs::msg::PointCloud2::Ptr &msg,
                         CloudIRLPtrs clouds, Detection3DArrayPtr detections);

  void
  save_point_reduction_counts(const sensor_msgs::msg::PointCloud2::Ptr &msg);

  void clear_markers(const std::string &frame_name);
  void clear_durations();
  std::string filename;

  int64_t normalization_duration_count;
  int64_t conveyor_clusterization_duration_count;
  int64_t conveyor_classification_duration_count;
  int64_t density_segmentation_duration_count;
  int64_t supports_clusterization_duration_count;
  int64_t supports_classification_duration_count;
  int64_t estimation_duration_count;

  std::size_t original_points_count;
  std::size_t filter_further_than_5m_points_count;
  std::size_t filter_ground_points_count;
  std::size_t roi_points_count;
  int debug = 1;
};
