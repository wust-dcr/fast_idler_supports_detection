#  Copyright 2024 Jakub Delicat
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_directory = get_package_share_directory("fast_idler_supports_detection")

    bag_file_path = LaunchConfiguration("bag_file_path")

    rviz_path = LaunchConfiguration("rviz_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_file_path",
                default_value="/path/to/default/bag/file",
                description="Path to the ROS bag file",
            ),
            DeclareLaunchArgument(
                "rviz_path",
                default_value=f"{package_share_directory}/rviz/pointclouds.rviz",
                description="Path to the RViz configuration file",
            ),
            Node(
                package="fast_idler_supports_detection",
                executable="fast_idler_supports_detection",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "bag_file_path": bag_file_path,
                        "general.pointcloud_topic_name": "velodyne_points_5m_clamp",
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="point_cloud_tf",
                output="log",
                arguments=[
                    "0.410",
                    "0",
                    "1.350",
                    "0",
                    "0.454",
                    "0",
                    "base_link",
                    "velodyne",
                ],
            ),
            Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_path]),
            ExecuteProcess(cmd=["ros2", "bag", "play", bag_file_path], output="screen"),
        ]
    )
