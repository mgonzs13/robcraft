# Copyright (C) 2026 Miguel Ángel González Santamarta
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robot_ns = LaunchConfiguration("robot_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_ns_cmd = DeclareLaunchArgument(
        "robot_ns",
        default_value="robot_george_1",
        description="Robot namespace",
    )

    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (RobCraft) clock if True",
    )

    params_file = os.path.join(
        get_package_share_directory("robcraft_maze_demo"),
        "config",
        "ekf_gps.yaml",
    )

    def topic(name):
        return PathJoinSubstitution(["/", robot_ns, name])

    def frame(name):
        return PathJoinSubstitution([robot_ns, name])

    # Leaf rewrites apply to every section, so per-node frames use full
    # "section.ros__parameters.param" paths. navsat_transform_node has no
    # frame params (it takes them from the source odometry message), so only
    # the two EKF sections are rewritten.
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "imu_filter_madgwick_node.ros__parameters.world_frame": "enu",
        "imu_filter_madgwick_node.ros__parameters.fixed_frame": frame("odom"),
        "ekf_local.ros__parameters.map_frame": "world",
        "ekf_local.ros__parameters.odom_frame": frame("odom"),
        "ekf_local.ros__parameters.world_frame": frame("odom"),
        "ekf_local.ros__parameters.base_link_frame": frame("base_footprint"),
        "ekf_global.ros__parameters.map_frame": "world",
        "ekf_global.ros__parameters.odom_frame": frame("odom"),
        "ekf_global.ros__parameters.world_frame": "world",
        "ekf_global.ros__parameters.base_link_frame": frame("base_footprint"),
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    navsat_cmd = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="log",
        parameters=[configured_params],
        remappings=[
            ("gps/fix", topic("gps")),
            ("imu", topic("imu")),
            ("odometry/filtered", topic("odom_filtered_gps")),
            ("odometry/gps", topic("gps/odom")),
        ],
    )

    ekf_local_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="log",
        parameters=[configured_params],
        remappings=[
            ("odom", topic("odom")),
            ("imu", topic("imu")),
            ("odometry/filtered", topic("odom_filtered")),
        ],
    )

    ekf_global_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="log",
        parameters=[configured_params],
        remappings=[
            ("odometry/gps", topic("gps/odom")),
            ("odom", topic("odom_filtered")),
            ("imu", topic("imu")),
            ("odometry/filtered", topic("odom_filtered_gps")),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(robot_ns_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(navsat_cmd)
    ld.add_action(ekf_local_cmd)
    ld.add_action(ekf_global_cmd)

    return ld
