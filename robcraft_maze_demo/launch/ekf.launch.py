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
        "ekf.yaml",
    )

    def topic(name):
        return PathJoinSubstitution(["/", robot_ns, name])

    def frame(name):
        return PathJoinSubstitution([robot_ns, name])

    param_substitutions = {
        "use_sim_time": use_sim_time,
        # The EKF owns the odom -> base_footprint TF (the sim runs with
        # --no-odom-tf), so the filtered odometry lives in the shared `odom`
        # frame. Rewriting these to `odom_filtered` leaves the `odom` frame
        # unpublished, which breaks rtabmap's RGB-D odometry TF lookups.
        "odom_frame": frame("odom"),
        "world_frame": frame("odom"),
        "base_link_frame": frame("base_footprint"),
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="log",
        parameters=[configured_params],
        remappings=[
            ("odometry/filtered", topic("odom_filtered")),
            ("odometry/imu", topic("imu")),
            ("odometry/visual", topic("odom_rgbd")),
            ("odometry/wheel", topic("odom")),
        ],
    )

    ld = LaunchDescription()

    ld.add_action(robot_ns_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(ekf_cmd)

    return ld
