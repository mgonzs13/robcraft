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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robcraft")
    default_world = os.path.join(share, "worlds", "maze.world")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Path to a .world file to load",
    )

    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="false",
        description="Whether to publish the shared world (map) TF frame",
    )

    publish_odom_tf_arg = DeclareLaunchArgument(
        "publish_odom_tf",
        default_value="true",
        description="Whether to publish the robot odom -> base_footprint TF",
    )

    texture_size_arg = DeclareLaunchArgument(
        "texture_size",
        default_value="256",
        description="Shared terrain/building texture size: 256, 512, or 1024",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run the simulator without a visible window",
    )

    world_frame_flag = PythonExpression(
        [
            "'--world-frame' if '",
            LaunchConfiguration("world_frame"),
            "' == 'true' else ''",
        ]
    )

    robot_tf_flag = PythonExpression(
        [
            "'--no-odom-tf' if '",
            LaunchConfiguration("publish_odom_tf"),
            "' == 'false' else ''",
        ]
    )

    headless_flag = PythonExpression(
        [
            "'--headless' if '",
            LaunchConfiguration("headless"),
            "' == 'true' else ''",
        ]
    )

    node = Node(
        package="robcraft",
        executable="robcraft",
        arguments=[
            LaunchConfiguration("world"),
            world_frame_flag,
            robot_tf_flag,
            headless_flag,
            "--texture-size",
            LaunchConfiguration("texture_size"),
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            world_frame_arg,
            publish_odom_tf_arg,
            texture_size_arg,
            headless_arg,
            node,
        ]
    )
