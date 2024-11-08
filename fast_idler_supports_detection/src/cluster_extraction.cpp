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

#include <fast_idler_supports_detection/cluster_extraction.hpp>

CloudIRLPtrs ClusterExtraction::euclidean(CloudIRPtr cloud, double tolerance,
                                          std::size_t min_size,
                                          std::size_t max_size) const {
  CloudIRPtr cloud_filtered(cloud);
  pcl::search::KdTree<PointIR>::Ptr tree(new pcl::search::KdTree<PointIR>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointIR> ec;
  ec.setClusterTolerance(tolerance); // 2cm
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  int j = 0;
  CloudIRLPtrs clouds;
  for (const auto &cluster : cluster_indices) {
    CloudIRLPtr cloud_cluster(new CloudIRL);
    for (const auto &idx : cluster.indices) {
      PointIRL point;
      point.x = (*cloud_filtered)[idx].x;
      point.y = (*cloud_filtered)[idx].y;
      point.z = (*cloud_filtered)[idx].z;
      point.intensity = (*cloud_filtered)[idx].intensity;
      point.ring = (*cloud_filtered)[idx].ring;
      point.label = j * 10;
      cloud_cluster->push_back(point);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clouds.push_back(cloud_cluster);
    ++j;
  }
  return clouds;
}

CloudIRLPtrs ClusterExtraction::euclidean(CloudIRPtr cloud) const {
  return euclidean(cloud, euclidean_tolerance, euclidean_min_size,
                   euclidean_max_size);
}
