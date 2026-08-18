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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("robcraft_maze_demo")
    launch_dir = os.path.join(pkg, "launch")

    robot_ns = LaunchConfiguration("robot_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

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

    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Whether launch rviz2",
    )

    rviz_config = os.path.join(pkg, "rviz", "ekf_gps_mapping_view.rviz")

    ekf_gps_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "ekf_gps.launch.py")),
        launch_arguments={
            "robot_ns": robot_ns,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rtabmap_lidar3d_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "rtabmap_lidar3d.launch.py")
        ),
        launch_arguments={
            "robot_ns": robot_ns,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(PythonExpression(["'", launch_rviz, "' == 'True'"])),
    )

    ld = LaunchDescription()

    ld.add_action(robot_ns_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(launch_rviz_cmd)

    ld.add_action(ekf_gps_cmd)
    ld.add_action(rtabmap_lidar3d_cmd)
    ld.add_action(rviz_cmd)

    return ld
