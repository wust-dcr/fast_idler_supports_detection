cmake_minimum_required(VERSION 3.8)
project(fast_idler_supports_detection)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(vision_msgs REQUIRED)

set(CMAKE_BUILD_TYPE Release)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)

add_library(detection_library SHARED
  src/utils.cpp
  src/cluster_extraction.cpp
  src/point_types.cpp
)

target_include_directories(
  detection_library
  PUBLIC
  include
)

target_link_libraries(detection_library
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(detection_library
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  vision_msgs
)

install(TARGETS
  detection_library
  DESTINATION lib/
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  detection_library
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  vision_msgs
)

install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}
  src/fast_idler_supports_detection.cpp
  src/fast_idler_supports_detection_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
  vision_msgs
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)


target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} detection_library)

install(TARGETS
${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
