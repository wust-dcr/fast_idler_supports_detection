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
#define PCL_XYZ_POINT_TYPES                                                    \
  PCL_XYZ_POINT_TYPES(pcl::PointXYZIR)                                         \
  (pcl::PointXYZIRL)
#include <pcl/point_types.h>

namespace pcl {
struct PointXYZIR {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  friend std::ostream &operator<<(std::ostream &os, const PointXYZIR &p);
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring, ring))

namespace pcl {
struct PointXYZIRL {
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  uint16_t label;
  friend std::ostream &operator<<(std::ostream &os, const PointXYZIR &p);
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(uint16_t, ring,
                                                 ring)(uint16_t, label, label))
