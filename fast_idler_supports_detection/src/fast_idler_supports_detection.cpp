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

#include <fast_idler_supports_detection/fast_idler_supports_detection.hpp>
#include <fstream>

FastIdlerSupportsDetection::FastIdlerSupportsDetection()
    : Node("fast_idler_supports_detection") {
  declare_parameters();
  get_parameters();
  create_rclcpp_instances();
}

void FastIdlerSupportsDetection::declare_parameters() {
  // FIXME: Double parameters are strings to work with rqt dynamic reconfigure.
  declare_parameter("general.filename", "output.yaml");
  declare_parameter("general.debug", "1");

  declare_parameter("general.pointcloud_topic_name", "/velodyne_points");
  declare_parameter("general.transform.roll", "-0.09");
  declare_parameter("general.transform.pitch", "0.55");
  declare_parameter("general.transform.yaw", "-0.06");
  declare_parameter("general.ground_level_height", "0.07");

  declare_parameter("general.transform.x", "0.0");
  declare_parameter("general.transform.y", "0.0");
  declare_parameter("general.transform.z", "1.35");

  declare_parameter("general.tunnel.height", "1.5");
  declare_parameter("general.tunnel.width", "5.00");
  declare_parameter("general.tunnel.length", "5.00");

  declare_parameter("general.forward.histogram.resolution", "0.1");
  declare_parameter("general.forward.histogram.min", "30");
  declare_parameter("general.forward.histogram.max", "30000");
  declare_parameter("general.forward.histogram.column_density_threshold", "30");

  declare_parameter("general.ground.histogram.resolution", "0.1");
  declare_parameter("general.ground.histogram.min", "120");
  declare_parameter("general.ground.histogram.max", "350");
  declare_parameter("general.ground.histogram.a", "3.5");
  declare_parameter("general.ground.histogram.slope", "10");
  declare_parameter("general.ground.histogram.range", "3");

  declare_parameter("outlier_remover.radius_outlier.neighbors_count", "8");
  declare_parameter("outlier_remover.radius_outlier.radius", "0.1");

  declare_parameter("conveyor_candidates_clusteler.euclidean.tolerance", "0.5");
  declare_parameter("conveyor_candidates_clusteler.euclidean.min_size", "100");
  declare_parameter("conveyor_candidates_clusteler.euclidean.max_size", "3000");

  declare_parameter("support_candidates_clusteler.euclidean.tolerance", "0.3");
  declare_parameter("support_candidates_clusteler.euclidean.min_size", "20");
  declare_parameter("support_candidates_clusteler.euclidean.max_size", "500");
}

void FastIdlerSupportsDetection::get_parameters() {
  debug = std::stoi(get_parameter("general.debug").as_string());
  filename = get_parameter("general.filename").as_string();
  pointcloud_topic_name =
      get_parameter("general.pointcloud_topic_name").as_string();
  roll = std::stod(get_parameter("general.transform.roll").as_string());
  pitch = std::stod(get_parameter("general.transform.pitch").as_string());
  yaw = std::stod(get_parameter("general.transform.yaw").as_string());

  x = std::stod(get_parameter("general.transform.x").as_string());
  y = std::stod(get_parameter("general.transform.y").as_string());
  z = std::stod(get_parameter("general.transform.z").as_string());

  tunnel_width = std::stod(get_parameter("general.tunnel.width").as_string());
  tunnel_height = std::stod(get_parameter("general.tunnel.height").as_string());
  tunnel_length = std::stod(get_parameter("general.tunnel.length").as_string());
  ground_level_height =
      std::stod(get_parameter("general.ground_level_height").as_string());

  forward_resolution = std::stod(
      get_parameter("general.forward.histogram.resolution").as_string());
  forward_histogram_min =
      std::stoi(get_parameter("general.forward.histogram.min").as_string());
  forward_histogram_max =
      std::stoi(get_parameter("general.forward.histogram.max").as_string());
  forward_column_density_threshold = std::stoi(
      get_parameter("general.forward.histogram.column_density_threshold")
          .as_string());

  ground_resolution = std::stod(
      get_parameter("general.ground.histogram.resolution").as_string());
  ground_histogram_min =
      std::stoi(get_parameter("general.ground.histogram.min").as_string());
  ground_histogram_max =
      std::stoi(get_parameter("general.ground.histogram.max").as_string());
  ground_histogram_a =
      std::stoi(get_parameter("general.ground.histogram.a").as_string());
  slope =
      std::stoi(get_parameter("general.ground.histogram.slope").as_string());
  range =
      std::stoi(get_parameter("general.ground.histogram.range").as_string());

  conveyor_candidates_clusteler.euclidean_tolerance = std::stod(
      get_parameter("conveyor_candidates_clusteler.euclidean.tolerance")
          .as_string());
  conveyor_candidates_clusteler.euclidean_max_size = std::stoi(
      get_parameter("conveyor_candidates_clusteler.euclidean.max_size")
          .as_string());
  conveyor_candidates_clusteler.euclidean_min_size = std::stoi(
      get_parameter("conveyor_candidates_clusteler.euclidean.min_size")
          .as_string());

  supports_candidates_clusteler.euclidean_tolerance = std::stod(
      get_parameter("support_candidates_clusteler.euclidean.tolerance")
          .as_string());
  supports_candidates_clusteler.euclidean_max_size =
      std::stoi(get_parameter("support_candidates_clusteler.euclidean.max_size")
                    .as_string());
  supports_candidates_clusteler.euclidean_min_size =
      std::stoi(get_parameter("support_candidates_clusteler.euclidean.min_size")
                    .as_string());
}

void FastIdlerSupportsDetection::create_rclcpp_instances() {
  transformed_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("inz/rotated", 10);
  distance_filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "inz/velodyne_points_5m_clamp", 10);

  without_ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "inz/without_ground_pub_", 10);

  forward_hist_filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "inz/forward_hist_filtered_pub_", 10);
  top_hist_filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "inz/top_hist_filtered_pub_", 10);
  clustered_conveyors_candidates_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
          "inz/clustered_conveyors_candidates", 10);
  clustered_conveyors_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "inz/clustered_candidates", 10);
  clustered_supports_candidates_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
          "inz/clustered_supports_candidates", 10);
  merged_density_clouds_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "inz/merged_density_clouds", 10);
  clustered_supports_candidates_velodyne_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
          "inz/clustered_supports_candidates_velodyne", 10);
  clustered_supports_candidates_base_link_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
          "inz/clustered_supports_candidates_base_link", 10);

  conveyors_detection_3d_pub_ =
      create_publisher<vision_msgs::msg::Detection3DArray>(
          "inz/conveyors_detection", 10);
  supports_detection_3d_pub_ =
      create_publisher<vision_msgs::msg::Detection3DArray>(
          "inz/supports_detection", 10);

  forward_density_histogram_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "inz/forward_density_histogram", 10);
  forward_density_clustered_histogram_pub_ =
      create_publisher<sensor_msgs::msg::Image>(
          "inz/forward_density_clustered_histogram", 10);

  ground_density_histogram_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "inz/ground_density_histogram", 10);
  ground_density_clustered_histogram_pub_ =
      create_publisher<sensor_msgs::msg::Image>(
          "inz/ground_density_clustered_histogram", 10);

  using std::placeholders::_1;
  const std::string &topic_name = pointcloud_topic_name;
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_name, 10,
      std::bind(&FastIdlerSupportsDetection::lidar_callback, this, _1));
}

void FastIdlerSupportsDetection::lidar_callback(
    const rclcppCloudSharedPtr msg) {
  auto start = std::chrono::high_resolution_clock::now();
  clear_durations();
  // get_parameters();
  RCLCPP_DEBUG(get_logger(), "Pointcloud callback.");
  if (msg->width * msg->height == 0) {
    RCLCPP_WARN(get_logger(), "Empty pointcloud skipping...");
    save_data_to_yaml(msg, {}, nullptr);

    auto end = std::chrono::high_resolution_clock::now();
    auto count =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();

    RCLCPP_INFO_STREAM(get_logger(), "Callback took: " << count / 1e6);
    return;
  }

  auto normalization_start = std::chrono::high_resolution_clock::now();

  auto frame = msg->header.frame_id;
  auto cloud_raw = pcl_utils::convert_point_cloud2_to_cloud_ptr<PointIR>(msg);
  original_points_count = cloud_raw->size();

  auto cloud = pcl_utils::remove_far_points_from_ros2bag_converter_bug<PointIR>(
      cloud_raw, 5.0);
  filter_further_than_5m_points_count = cloud->size();
  // Ground Filtering
  Eigen::Vector3d normal_vec;
  double ground_height;

  CloudIRPtr cloud_for_ground_detection(new CloudIR);
  for (const auto &point : cloud->points) {
    if (std::abs(point.y) < tunnel_width / 2.0) {
      cloud_for_ground_detection->push_back(point);
    }
  }
  auto [ground, without_ground] = filter_ground_and_get_normal_and_height(
      cloud_for_ground_detection, pcl::SACMODEL_PLANE, 800, ground_level_height,
      std::ref(normal_vec), std::ref(ground_height));

  filter_ground_points_count = without_ground->size();

  Eigen::Vector3d rpy;
  auto aligned_cloud =
      align_to_normal(without_ground, normal_vec, ground_height, std::ref(rpy));

  auto normalization_end = std::chrono::high_resolution_clock::now();

  normalization_duration_count =
      std::chrono::duration_cast<std::chrono::microseconds>(normalization_end -
                                                            normalization_start)
          .count();

  auto density_segmentation_start = std::chrono::high_resolution_clock::now();

  CloudIRLPtr merged_conveyors(new CloudIRL);
  for (const auto &point : aligned_cloud->points) {
    PointIRL p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    p.ring = point.ring;
    p.intensity = point.intensity;
    p.label = 0;
    merged_conveyors->push_back(p);
  }

  merged_conveyors->width = merged_conveyors->size();
  merged_conveyors->height = 1;
  auto histogram = create_histogram(merged_conveyors, forward_resolution);

  if (not histogram.data.size() or not histogram.data[0].size()) {
    RCLCPP_WARN(get_logger(), "Cannot create a front histogram.");
    auto density_segmentation_end = std::chrono::high_resolution_clock::now();
    density_segmentation_duration_count =
        std::chrono::duration_cast<std::chrono::microseconds>(
            density_segmentation_end - density_segmentation_start)
            .count();
    clear_markers(frame);
    save_data_to_yaml(msg, {}, nullptr);

    auto end = std::chrono::high_resolution_clock::now();
    auto count =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();

    RCLCPP_INFO_STREAM(get_logger(), "Callback took: " << count / 1e6);
    return;
  }

  auto clustered_histogram = threshold_histogram(
      histogram, forward_histogram_min, forward_histogram_max);
  auto low_density_cloud =
      filter_with_density_on_x_image(merged_conveyors, clustered_histogram);

  auto rotated_for_ground_histogram =
      pcl_utils::rotate<PointIRL>(merged_conveyors, 0.0, -M_PI / 2.0, 0.0);
  auto ground_histogram =
      create_top_histogram(rotated_for_ground_histogram, ground_resolution);

  if (not ground_histogram.data.size() or not ground_histogram.data[0].size()) {
    RCLCPP_WARN(get_logger(), "Cannot create a top histogram.");
    auto density_segmentation_end = std::chrono::high_resolution_clock::now();
    density_segmentation_duration_count =
        std::chrono::duration_cast<std::chrono::microseconds>(
            density_segmentation_end - density_segmentation_start)
            .count();
    clear_markers(frame);
    save_data_to_yaml(msg, {}, nullptr);

    auto end = std::chrono::high_resolution_clock::now();
    auto count =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();

    RCLCPP_INFO_STREAM(get_logger(), "Callback took: " << count / 1e6);
    return;
  }

  auto clustered_ground_histogram =
      segment_local_peeks(ground_histogram, slope, range);
  auto high_density_top_cloud = filter_with_density_on_z_image(
      merged_conveyors, clustered_ground_histogram);

  auto merged_density_cloud{pcl_utils::remove_simillar_points<PointIRL>(
      {high_density_top_cloud, low_density_cloud}, 0.0001)};

  auto density_segmentation_end = std::chrono::high_resolution_clock::now();
  density_segmentation_duration_count =
      std::chrono::duration_cast<std::chrono::microseconds>(
          density_segmentation_end - density_segmentation_start)
          .count();

  // ============================================
  auto supports_clusterization_start =
      std::chrono::high_resolution_clock::now();

  // TODO: create function remove label
  auto pc2 = std::make_shared<sensor_msgs::msg::PointCloud2>(
      pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIRL>(
          merged_density_cloud, frame, this));
  auto top_cloud = pcl_utils::convert_point_cloud2_to_cloud_ptr<PointIR>(pc2);

  auto clustered_supports_candidates =
      supports_candidates_clusteler.euclidean(top_cloud);

  auto supports_clusterization_end = std::chrono::high_resolution_clock::now();
  supports_clusterization_duration_count =
      std::chrono::duration_cast<std::chrono::microseconds>(
          supports_clusterization_end - supports_clusterization_start)
          .count();

  if (not clustered_supports_candidates.size()) {
    RCLCPP_WARN(get_logger(),
                "Cannot find any support candidate! Skipping pointcloud");
    clear_markers(frame);
    save_data_to_yaml(msg, {}, nullptr);

    auto end = std::chrono::high_resolution_clock::now();
    auto count =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();

    RCLCPP_INFO_STREAM(get_logger(), "Callback took: " << count / 1e6);
    return;
  }

  // ============================================
  auto supports_classification_start =
      std::chrono::high_resolution_clock::now();

  auto merged_supports_candidates =
      pcl_utils::merge_clouds<PointIRL>(clustered_supports_candidates);
  auto supports_candidates_detection_3d_msg =
      detect_supports(clustered_supports_candidates, frame);

  auto supports_classification_end = std::chrono::high_resolution_clock::now();
  supports_classification_duration_count =
      std::chrono::duration_cast<std::chrono::microseconds>(
          supports_classification_end - supports_classification_start)
          .count();

  auto estimation_start = std::chrono::high_resolution_clock::now();

  CloudIRLPtrs base_linked_clouds;
  for (const auto &cloud : clustered_supports_candidates) {
    auto to_velodyne =
        pcl_utils::translate<pcl::PointXYZIRL>(cloud, 0, 0, -ground_height);
    auto to_velodyne_rotated = pcl_utils::rotate<pcl::PointXYZIRL>(
        to_velodyne, -rpy[0], -rpy[1], -rpy[2] - 0.06);
    auto to_base_link_rotated =
        pcl_utils::rotate<pcl::PointXYZIRL>(to_velodyne_rotated, 0, 0.454, 0);
    auto to_base_link = pcl_utils::translate<pcl::PointXYZIRL>(
        to_base_link_rotated, 0.410, 0, 1.350);
    base_linked_clouds.push_back(to_base_link);
  }
  auto base_linked = pcl_utils::merge_clouds<PointIRL>(base_linked_clouds);

  auto estimation_end = std::chrono::high_resolution_clock::now();
  estimation_duration_count =
      std::chrono::duration_cast<std::chrono::microseconds>(estimation_end -
                                                            estimation_start)
          .count();

  // Publish
  if (debug) {
    distance_filtered_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIR>(cloud, frame,
                                                              this));
    without_ground_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIR>(without_ground,
                                                              frame, this));
    transformed_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIR>(aligned_cloud,
                                                              frame, this));

    forward_density_histogram_pub_->publish(
        create_image_from_histogram(histogram));
    forward_density_clustered_histogram_pub_->publish(
        create_image_from_histogram(clustered_histogram));
    forward_hist_filtered_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIRL>(
            low_density_cloud, frame, this));

    ground_density_clustered_histogram_pub_->publish(
        create_image_from_histogram(clustered_ground_histogram));
    ground_density_histogram_pub_->publish(
        create_image_from_histogram(ground_histogram));
    top_hist_filtered_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIRL>(
            high_density_top_cloud, frame, this));
    merged_density_clouds_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIRL>(
            merged_density_cloud, frame, this));

    clustered_supports_candidates_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIRL>(
            merged_supports_candidates, frame, this));

    auto supports_in_base_frame =
        detect_supports(base_linked_clouds, "base_link");
    supports_detection_3d_pub_->publish(*supports_in_base_frame);

    // clustered_supports_candidates_velodyne_pub_->publish(
    //     pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIRL>(to_velodyne_rotated,
    //     frame, this));
    clustered_supports_candidates_base_link_pub_->publish(
        pcl_utils::convert_cloud_ptr_to_point_cloud2<PointIRL>(
            base_linked, "base_link", this));
  }

  save_data_to_yaml(msg, base_linked_clouds,
                    supports_candidates_detection_3d_msg);
  save_point_reduction_counts(msg);

  auto end = std::chrono::high_resolution_clock::now();
  auto count =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count();

  RCLCPP_INFO_STREAM(get_logger(),
                     "Callback took: " << count / 1e6 << " for pointcloud at "
                                       << msg->header.stamp.sec << "."
                                       << msg->header.stamp.nanosec);
}

void FastIdlerSupportsDetection::clear_markers(const std::string &frame_name) {
  vision_msgs::msg::Detection3DArray empty;
  empty.header.frame_id = frame_name;
  empty.header.stamp = get_clock()->now();
  conveyors_detection_3d_pub_->publish(empty);
  supports_detection_3d_pub_->publish(empty);
}

Histogram FastIdlerSupportsDetection::create_top_histogram(CloudIRLPtr cloud,
                                                           double resolution) {
  Histogram histogram_image;
  auto compare_y = [](const PointIRL lhs, const PointIRL rhs) {
    return lhs.y < rhs.y;
  };
  auto compare_z = [](const PointIRL lhs, const PointIRL rhs) {
    return lhs.z < rhs.z;
  };

  auto point_with_max_y =
      *std::max_element(cloud->points.begin(), cloud->points.end(), compare_y);
  auto point_with_max_z =
      *std::max_element(cloud->points.begin(), cloud->points.end(), compare_z);

  if (2 * point_with_max_y.y < 0 or point_with_max_z.z < 0) {
    RCLCPP_WARN_STREAM(
        get_logger(),
        "Skipping creating histogram under the surface OXY. Width: "
            << 2 * point_with_max_y.y << " height: " << point_with_max_z.z);
    return histogram_image;
  }

  histogram_image.resolution = resolution;
  histogram_image.width = std::abs(2 * point_with_max_y.y);
  histogram_image.height = std::abs(point_with_max_z.z);

  histogram_image.image_width = static_cast<std::size_t>(
      histogram_image.width / histogram_image.resolution);
  histogram_image.image_height = static_cast<std::size_t>(
      histogram_image.height / histogram_image.resolution);

  if (histogram_image.image_width == 0 or histogram_image.image_height == 0) {
    return histogram_image;
  }

  histogram_image.data.resize(histogram_image.image_height);

  for (auto &column : histogram_image.data) {
    column.resize(histogram_image.image_width);
  }

  std::list<std::size_t> banned_height;
  std::list<std::size_t> banned_width;

  for (const auto &point : cloud->points) {
    const auto image_width_pos = static_cast<std::size_t>(
        (histogram_image.width / 2 - point.y) / resolution);
    const auto image_height_pos =
        static_cast<std::size_t>(point.z / histogram_image.resolution);

    if (image_width_pos >= histogram_image.image_width or
        image_height_pos >= histogram_image.image_height) {
      continue;
    }

    auto found_height = std::find(banned_height.begin(), banned_height.end(),
                                  image_height_pos) != std::end(banned_height);
    if ((std::find(banned_width.begin(), banned_width.end(), image_width_pos) !=
         std::end(banned_width)) and
        found_height) {
      continue;
    }

    if (std::abs(point.x) > 0.7) {
      histogram_image.data[image_height_pos][image_width_pos] = 0;
      banned_height.push_back(image_height_pos);
      banned_width.push_back(image_width_pos);
      RCLCPP_DEBUG_STREAM(get_logger(),
                          "Found pillar z: " << point.x
                                             << " at: " << image_height_pos
                                             << " " << image_width_pos);
      continue;
    }

    ++histogram_image.data[image_height_pos][image_width_pos];
  }
  return histogram_image;
}

Histogram FastIdlerSupportsDetection::create_histogram(CloudIRLPtr cloud,
                                                       double resolution) {
  Histogram histogram_image;
  auto compare_y = [](const PointIRL lhs, const PointIRL rhs) {
    return lhs.y < rhs.y;
  };
  auto compare_z = [](const PointIRL lhs, const PointIRL rhs) {
    return lhs.z < rhs.z;
  };

  auto point_with_max_y =
      *std::max_element(cloud->points.begin(), cloud->points.end(), compare_y);
  auto point_with_max_z =
      *std::max_element(cloud->points.begin(), cloud->points.end(), compare_z);

  if (2 * point_with_max_y.y < 0 or point_with_max_z.z < 0) {
    RCLCPP_WARN_STREAM(
        get_logger(),
        "Skipping creating histogram under the surface OXY. Width: "
            << 2 * point_with_max_y.y << " height: " << point_with_max_z.z);
    return histogram_image;
  }

  histogram_image.resolution = resolution;
  histogram_image.width = std::abs(2 * point_with_max_y.y);
  histogram_image.height = std::abs(point_with_max_z.z);

  histogram_image.image_width = static_cast<std::size_t>(
      histogram_image.width / histogram_image.resolution);
  histogram_image.image_height = static_cast<std::size_t>(
      histogram_image.height / histogram_image.resolution);

  if (histogram_image.image_width == 0 or histogram_image.image_height == 0) {
    return histogram_image;
  }

  histogram_image.data.resize(histogram_image.image_height);

  for (auto &column : histogram_image.data) {
    column.resize(histogram_image.image_width);
  }

  for (const auto &point : cloud->points) {
    const auto image_width_pos = static_cast<std::size_t>(
        (histogram_image.width / 2 - point.y) / resolution);
    const auto image_height_pos =
        static_cast<std::size_t>(point.z / histogram_image.resolution);

    if (image_width_pos >= histogram_image.image_width or
        image_height_pos >= histogram_image.image_height) {
      continue;
    }

    ++histogram_image.data[image_height_pos][image_width_pos];
  }

  return histogram_image;
}

sensor_msgs::msg::Image FastIdlerSupportsDetection::create_image_from_histogram(
    const Histogram &histogram) {
  sensor_msgs::msg::Image image_msg;
  image_msg.height = histogram.image_height;
  image_msg.width = histogram.image_width;
  image_msg.encoding = "mono8";

  if (image_msg.height == 0 or image_msg.width == 0) {
    return image_msg;
  }

  image_msg.step = image_msg.width;
  image_msg.data.resize(image_msg.height * image_msg.step, 0);
  // TODO: another function
  auto max_element = std::numeric_limits<std::size_t>::min();
  for (const auto &col : histogram.data) {
    const auto col_max = *std::max_element(col.begin(), col.end());
    max_element = std::max(col_max, max_element);
  }
  if (not max_element) {
    return image_msg;
  }

  for (std::size_t i = 0; i < image_msg.height; ++i) {
    for (std::size_t j = 0; j < histogram.data[i].size(); ++j) {
      uint8_t intensity =
          static_cast<uint8_t>(255 * histogram.data[i][j] / max_element);
      image_msg.data[(image_msg.height - i - 1) * image_msg.step + j] =
          intensity;
    }
  }

  return image_msg;
}

CloudIRLPtr FastIdlerSupportsDetection::filter_with_density_on_x_image(
    CloudIRLPtr cloud, const Histogram &histogram) {
  const auto width = histogram.width;
  const auto height = histogram.height;
  const auto image_width = histogram.image_width;
  const auto image_height = histogram.image_height;
  const auto resolution = histogram.resolution;

  auto low_density_cloud = CloudIRLPtr(new CloudIRL);
  for (const auto &point : cloud->points) {
    const auto image_width_pos =
        static_cast<std::size_t>((width / 2 - point.y) / resolution);
    const auto image_height_pos =
        static_cast<std::size_t>(point.z / resolution);

    if (image_width_pos >= image_width or image_height_pos >= image_height) {
      continue;
    }
    if (histogram.data[image_height_pos][image_width_pos]) {
      low_density_cloud->points.push_back(point);
    }
  }
  low_density_cloud->width = low_density_cloud->size();
  low_density_cloud->height = 1;
  return low_density_cloud;
}

CloudIRLPtr FastIdlerSupportsDetection::filter_with_density_on_z_image(
    CloudIRLPtr cloud, const Histogram &histogram) {
  const auto width = histogram.width;
  const auto height = histogram.height;
  const auto image_width = histogram.image_width;
  const auto image_height = histogram.image_height;
  const auto resolution = histogram.resolution;

  auto low_density_cloud = CloudIRLPtr(new CloudIRL);
  for (const auto &point : cloud->points) {
    const auto image_width_pos =
        static_cast<std::size_t>((width / 2 - point.y) / resolution);
    const auto image_length_pos =
        static_cast<std::size_t>(point.x / resolution);

    if (image_width_pos >= image_width or image_length_pos >= image_height) {
      continue;
    }
    if (histogram.data[image_length_pos][image_width_pos]) {
      low_density_cloud->points.push_back(point);
    }
  }
  low_density_cloud->width = low_density_cloud->size();
  low_density_cloud->height = 1;
  return low_density_cloud;
}

Histogram FastIdlerSupportsDetection::threshold_histogram(
    const Histogram &histogram, std::size_t min, std::size_t max) {
  auto clustered_histogram(histogram);
  for (auto &col : clustered_histogram.data) {
    for (auto &density : col) {
      auto clamped_density = std::clamp(density, min, max);
      if (density != clamped_density and density != 0) {
        density = 0;
      }
    }
  }
  return clustered_histogram;
}

Histogram FastIdlerSupportsDetection::remove_low_density_columns(
    const Histogram &histogram, std::size_t threshold) {
  auto clustered_histogram(histogram);

  for (std::size_t i = 1; i < clustered_histogram.data[0].size() - 1; ++i) {
    for (std::size_t j = 0; j < clustered_histogram.data.size(); ++j) {
      if (clustered_histogram.data[j][i + 1] != 0 and
          clustered_histogram.data[j][i - 1] != 0) {
        clustered_histogram.data[j][i] = 0;
      }
    }
  }
  return clustered_histogram;
}

BoundingBoxArrayPtr
FastIdlerSupportsDetection::make_bounding_boxes_from_pointclouds(
    const CloudIRLPtrs &clustered_supports_candidates,
    const std::string &frame_name) {
  BoundingBoxArrayPtr bounding_boxes(new vision_msgs::msg::BoundingBox3DArray);
  for (const auto &leg : clustered_supports_candidates) {
    vision_msgs::msg::BoundingBox3D bounding_box;
    const float &max_value = std::numeric_limits<float>::max();
    const float &min_value = -std::numeric_limits<float>::max();
    Point max_coords{min_value, min_value, min_value};
    Point min_coords{max_value, max_value, max_value};
    for (const auto &point : leg->points) {
      max_coords.x = std::max(point.x, max_coords.x);
      max_coords.y = std::max(point.y, max_coords.y);
      max_coords.z = std::max(point.z, max_coords.z);

      min_coords.x = std::min(point.x, min_coords.x);
      min_coords.y = std::min(point.y, min_coords.y);
      min_coords.z = std::min(point.z, min_coords.z);
    }

    bounding_box.size.x = std::abs(max_coords.x - min_coords.x);
    bounding_box.size.y = std::abs(max_coords.y - min_coords.y);
    bounding_box.size.z = std::abs(max_coords.z - min_coords.z);

    bounding_box.center.position.x = min_coords.x + bounding_box.size.x / 2.0;
    bounding_box.center.position.y = min_coords.y + bounding_box.size.y / 2.0;
    bounding_box.center.position.z = min_coords.z + bounding_box.size.z / 2.0;

    bounding_boxes->boxes.push_back(bounding_box);
  }
  bounding_boxes->header.frame_id = frame_name;
  bounding_boxes->header.stamp = get_clock()->now();
  return bounding_boxes;
}

vision_msgs::msg::ObjectHypothesisWithPose
FastIdlerSupportsDetection::score_conveyor(
    const vision_msgs::msg::BoundingBox3D bbox) {
  // Left and right side conveyors
  double conveyor_height = 0.6 - ground_level_height;
  double conveyor_position_z = conveyor_height / 2.0 + ground_level_height;

  std::string class_name = "conveyor_0_6m";
  if (bbox.center.position.y < 0) {
    conveyor_height = 0.7 - ground_level_height;
    conveyor_position_z = conveyor_height / 2.0 + ground_level_height;

    class_name = "conveyor_0_7m";
  }

  vision_msgs::msg::ObjectHypothesisWithPose object;
  auto z_error = std::abs(bbox.center.position.z - conveyor_position_z) /
                 conveyor_position_z;
  auto height_error = std::abs(bbox.size.z - conveyor_height) / conveyor_height;
  auto whole_error = z_error + height_error;

  object.hypothesis.score = std::max({0.0, 1.0 - whole_error});
  object.hypothesis.class_id = class_name;
  return object;
}

Detection3DArrayPtr FastIdlerSupportsDetection::detect_conveyors(
    const CloudIRLPtrs &clustered_supports_candidates,
    const std::string &frame_name) {
  auto bboxes = make_bounding_boxes_from_pointclouds(
      clustered_supports_candidates, frame_name);
  Detection3DArrayPtr detections(new vision_msgs::msg::Detection3DArray);
  detections->header.frame_id = frame_name;
  detections->header.stamp = get_clock()->now();

  int i = 0;
  for (const auto &bbox : bboxes->boxes) {
    vision_msgs::msg::Detection3D detection;
    detection.header = detections->header;
    detection.bbox = bbox;
    detection.results.push_back(score_conveyor(bbox));
    detections->detections.push_back(detection);
  }

  return detections;
}

Detection3DArrayPtr FastIdlerSupportsDetection::detect_supports(
    const CloudIRLPtrs &clustered_supports_candidates,
    const std::string &frame_name) {
  auto bboxes = make_bounding_boxes_from_pointclouds(
      clustered_supports_candidates, frame_name);
  Detection3DArrayPtr detections(new vision_msgs::msg::Detection3DArray);
  detections->header.frame_id = frame_name;
  detections->header.stamp = get_clock()->now();

  for (auto i = 0u; i < bboxes->boxes.size(); ++i) {
    vision_msgs::msg::Detection3D detection;
    auto &bbox = bboxes->boxes[i];
    auto &cloud = clustered_supports_candidates[i];
    detection.header = detections->header;
    detection.bbox = bbox;

    auto compare_z = [](const PointIRL lhs, const PointIRL rhs) {
      return lhs.z < rhs.z;
    };

    auto point_with_max_z = *std::max_element(cloud->points.begin(),
                                              cloud->points.end(), compare_z);
    auto point_with_min_z = *std::min_element(cloud->points.begin(),
                                              cloud->points.end(), compare_z);

    // Left and right side conveyors
    double support_height = 0.6;
    std::string class_name = "support_0_6m";
    if (point_with_max_z.y < 0) {
      support_height = 0.7;
      class_name = "support_0_7m";
    }

    vision_msgs::msg::ObjectHypothesisWithPose support_object;
    vision_msgs::msg::ObjectHypothesisWithPose top_object;
    vision_msgs::msg::ObjectHypothesisWithPose bottom_object;
    double top_score = 1.0;
    double bottom_score = 1.0;

    top_score = std::max(
        0.0001, 1 - std::abs(point_with_max_z.z - support_height) / 0.1);
    bottom_score = std::max(
        0.0001, 1 - std::abs(point_with_min_z.z - ground_level_height) / 0.1);

    support_object.hypothesis.score =
        2 * (top_score * bottom_score) / (top_score + bottom_score);
    top_object.hypothesis.score = top_score;
    bottom_object.hypothesis.score = bottom_score;

    support_object.hypothesis.class_id = class_name;
    top_object.hypothesis.class_id = class_name + "/top";
    bottom_object.hypothesis.class_id = class_name + "/bottom";

    detection.results.push_back(support_object);
    detection.results.push_back(top_object);
    detection.results.push_back(bottom_object);

    detections->detections.push_back(detection);
  }

  return detections;
}

std::pair<CloudIRPtr, CloudIRPtr>
FastIdlerSupportsDetection::filter_ground_and_get_normal_and_height(
    CloudIRPtr cloud, int sac_model, int iterations, double radius,
    Eigen::Vector3d &normal, double &ground_height, double eps) {
  // Segment ground
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  CloudIRPtr ground(new CloudIR);
  CloudIRPtr without_ground(new CloudIR);

  pcl::SACSegmentation<pcl::PointXYZIR> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(sac_model);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(radius);
  seg.setMaxIterations(iterations);
  seg.setEpsAngle(eps);
  pcl::ExtractIndices<pcl::PointXYZIR> extract;
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
    // TODO: check it above
    return {nullptr, nullptr};
  }
  extract.setNegative(true);
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.filter(*without_ground);
  // extract.setNegative(false);
  // extract.filter(*ground);

  normal << coefficients->values[0], coefficients->values[1],
      coefficients->values[2];
  ground_height = coefficients->values[3];
  return {ground, without_ground};
}

CloudIRPtr FastIdlerSupportsDetection::align_to_normal(
    CloudIRPtr cloud, const Eigen::Vector3d &normal, double ground_height,
    Eigen::Vector3d &rpy) {
  const auto &up_vector = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d axis = normal.cross(up_vector).normalized();
  float angle =
      acos(normal.dot(up_vector) / (normal.norm() * up_vector.norm()));

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = Eigen::AngleAxisd(angle, axis);
  RCLCPP_DEBUG_STREAM(get_logger(), "Rotation matrix: \n" << rotation_matrix);
  rpy = rotation_matrix.normalized().eulerAngles(2, 1, 0);
  auto transformed_cloud = pcl_utils::translate<pcl::PointXYZIR>(
      pcl_utils::rotate<pcl::PointXYZIR>(cloud, rpy[0], rpy[1], rpy[2] + 0.06),
      0, 0, ground_height);
  transformed_cloud->height = transformed_cloud->points.size();
  transformed_cloud->width = 1;
  return transformed_cloud;
}

void FastIdlerSupportsDetection::save_data_to_yaml(
    const sensor_msgs::msg::PointCloud2::Ptr &msg, CloudIRLPtrs clouds,
    Detection3DArrayPtr detections) {
  YAML::Node frame_node;
  YAML::Node yaml_node;
  std::size_t detected_count;
  // Dodać środek i prawdopodobieństwo.

  for (auto i = 0u; i < clouds.size(); ++i) {
    YAML::Node object_node;

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float mean_point_x = 0;
    float mean_point_y = 0;
    float mean_point_z = 0;
    for (auto const &point : clouds[i]->points) {
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      min_z = std::min(min_z, point.z);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
      max_z = std::max(max_z, point.z);
      mean_point_x += point.x;
      mean_point_y += point.y;
      mean_point_z += point.z;

      YAML::Node point_node;
      point_node["x"] = point.x;
      point_node["y"] = point.y;
      point_node["z"] = point.z;
      point_node["intensity"] = point.intensity;
      point_node["ring"] = point.ring;
      object_node["points"].push_back(point_node);
    }

    mean_point_x /= clouds[i]->size();
    mean_point_y /= clouds[i]->size();
    mean_point_z /= clouds[i]->size();

    object_node["box_size"]["x"] = max_x - min_x;
    object_node["box_size"]["y"] = max_y - min_y;
    object_node["box_size"]["z"] = max_z - min_z;

    object_node["box_center"]["x"] = min_x + (max_x - min_x) / 2.0;
    object_node["box_center"]["y"] = min_y + (max_y - min_y) / 2.0;
    object_node["box_center"]["z"] = min_z + (max_z - min_z) / 2.0;

    object_node["frame_id"] = "base_link";

    object_node["mean_point"]["x"] = mean_point_x;
    object_node["mean_point"]["y"] = mean_point_y;
    object_node["mean_point"]["z"] = mean_point_z;

    if (detections != nullptr) {
      object_node["score"] =
          detections->detections[i].results[0].hypothesis.score;
      object_node["score_bottom"] =
          detections->detections[i].results[1].hypothesis.score;
      object_node["score_top"] =
          detections->detections[i].results[2].hypothesis.score;
    } else {
      object_node["score"] = 0.0;
      object_node["score_bottom"] = 0.0;
      object_node["score_top"] = 0.0;
    }

    frame_node["objects"].push_back(object_node);
  }
  frame_node["timestamp"]["sec"] = msg->header.stamp.sec;
  frame_node["timestamp"]["nanosec"] = msg->header.stamp.nanosec;

  frame_node["durations"]["normalization"] = normalization_duration_count / 1e6;
  frame_node["durations"]["conveyor_clusterization"] =
      conveyor_clusterization_duration_count / 1e6;
  frame_node["durations"]["conveyor_classification"] =
      conveyor_classification_duration_count / 1e6;
  frame_node["durations"]["density_segmentation"] =
      density_segmentation_duration_count / 1e6;
  frame_node["durations"]["supports_clusterization"] =
      supports_clusterization_duration_count / 1e6;
  frame_node["durations"]["supports_classification"] =
      supports_classification_duration_count / 1e6;
  frame_node["durations"]["estimation"] = estimation_duration_count / 1e6;
  auto processing_duration =
      normalization_duration_count + density_segmentation_duration_count +
      conveyor_clusterization_duration_count +
      conveyor_classification_duration_count +
      supports_clusterization_duration_count +
      supports_classification_duration_count + estimation_duration_count;

  frame_node["durations"]["processing"] = processing_duration / 1e6;

  yaml_node.push_back(frame_node);

  auto start = std::chrono::high_resolution_clock::now();
  std::ofstream file(filename, std::ios::app);

  if (file.is_open()) {
    file << yaml_node << std::endl;
    auto end = std::chrono::high_resolution_clock::now();
    auto microseconds =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();
    RCLCPP_DEBUG_STREAM(get_logger(), "Data saved to file. Saving took: "
                                          << microseconds / 10e3 << "ms");
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot save data!");
  }
}

void FastIdlerSupportsDetection::save_point_reduction_counts(
    const sensor_msgs::msg::PointCloud2::Ptr &msg) {
  YAML::Node frame_node;
  YAML::Node yaml_node;
  frame_node["timestamp"]["sec"] = msg->header.stamp.sec;
  frame_node["timestamp"]["nanosec"] = msg->header.stamp.nanosec;

  frame_node["original_points_count"] = original_points_count;
  frame_node["filter_further_than_5m_points_count"] =
      filter_further_than_5m_points_count;
  frame_node["filter_ground_points_count"] = filter_ground_points_count;
  frame_node["roi_points_count"] = roi_points_count;
  yaml_node.push_back(frame_node);

  std::ofstream file(filename + "_counts", std::ios::app);

  if (file.is_open()) {
    file << yaml_node << std::endl;
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot save data!");
  }
}

Histogram FastIdlerSupportsDetection::segment_local_peeks(
    const Histogram &histogram, std::size_t slope, std::size_t range) {
  Histogram segmented_histogram(histogram);
  for (auto i = range; i < histogram.data.size() - range; ++i) {
    for (auto j = 0u; j < histogram.data[i].size(); ++j) {
      for (auto k = 1u; k <= range; ++k) {
        if (segmented_histogram.data[i][j] != 0 &&
            (histogram.data[i][j] < histogram[i - k][j] + slope ||
             histogram.data[i][j] < histogram.data[i + k][j] + slope)) {
          segmented_histogram.data[i][j] = 0;
        }
      }
    }
  }
  return segmented_histogram;
}

void FastIdlerSupportsDetection::clear_durations() {
  normalization_duration_count = 0;
  conveyor_clusterization_duration_count = 0;
  conveyor_classification_duration_count = 0;
  density_segmentation_duration_count = 0;
  supports_clusterization_duration_count = 0;
  supports_classification_duration_count = 0;
  estimation_duration_count = 0;
}
