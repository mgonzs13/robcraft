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
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _pick_params_file(context):
    """Select the Nav2 params YAML matching the sourced ROS 2 distro."""
    distro = os.environ.get("ROS_DISTRO", "")
    filename = (
        "nav2_params_jazzy.yaml" if "jazzy" in distro else "nav2_params_humble.yaml"
    )
    params_file = os.path.join(
        get_package_share_directory("robcraft_maze_demo"), "config", filename
    )
    return [SetLaunchConfiguration("params_file", params_file)]


def generate_launch_description():

    bringup_dir = get_package_share_directory("robcraft_maze_demo")
    launch_dir = os.path.join(bringup_dir, "launch/navigation")

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz", default_value="True", description="Whether launch rviz"
    )

    slam = LaunchConfiguration("slam")
    slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    map_yaml_file = LaunchConfiguration(
        "map", default=os.path.join(bringup_dir, "maps/maze", "map.yaml")
    )
    map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=map_yaml_file,
        description="Full path to map yaml file to load",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (RobCraft) clock if True",
    )

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x", default_value="6.98", description="Initial pose x"
    )

    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y", default_value="22.8", description="Initial pose y"
    )

    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_z_cmd = DeclareLaunchArgument(
        "initial_pose_z", default_value="0.0", description="Initial pose z"
    )

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw", default_value="-1.57", description="Initial pose yaw"
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup.launch.py")),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "slam": slam,
            "params_file": LaunchConfiguration("params_file"),
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_z": initial_pose_z,
            "initial_pose_yaw": initial_pose_yaw,
            # Robot george (robot_george_1): the nav2 config is bound to it.
            "cmd_vel_topic": "/robot_george_1/cmd_vel",
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(launch_rviz_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(map_yaml_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    ld.add_action(OpaqueFunction(function=_pick_params_file))

    ld.add_action(bringup_cmd)

    return ld
