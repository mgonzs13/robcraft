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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


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

    def topic(name):
        return PathJoinSubstitution(["/", robot_ns, name])

    def frame(name):
        return PathJoinSubstitution([robot_ns, name])

    parameters = [
        {
            # Output odometry in the EKF's base frame so the child_frame_id of
            # the odom message matches robot_localization's base_link_frame.
            "frame_id": frame("base_footprint"),
            "odom_frame_id": frame("odom"),
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "approx_sync": True,
            "approx_sync_max_interval": 0.01,
            "publish_tf": False,
            "wait_imu_to_init": False,
            "publish_null_when_lost": False,
            "qos": 1,
            "qos_camera_info": 1,
            "use_sim_time": use_sim_time,
            # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            # 0=Frame-to-Map (F2M), 1=Frame-to-Frame, ...
            "Odom/Strategy": "0",
            "Odom/ResetCountdown": "1",
            "Odom/Holonomic": "false",
            # 0=No filtering, 1=Kalman filtering, 2=Particle filtering
            "Odom/FilteringStrategy": "1",
            "Odom/ParticleSize": "500",
            "Odom/GuessMotion": "true",
            "Odom/AlignWithGround": "false",
            "OdomF2M/MaxSize": "5000",
            "OdomF2M/ScanMaxSize": "5000",
            # Motion estimation: 0=3D->3D, 1=3D->2D (PnP), 2=2D->2D
            "Vis/EstimationType": "1",
            "Vis/ForwardEstOnly": "true",
            # 8=GFTT/ORB
            "Vis/FeatureType": "8",
            "Vis/DepthAsMask": "true",
            "Vis/CorGuessWinSize": "40",
            "Vis/MaxFeatures": "0",
            "Vis/MinDepth": "0.0",
            "Vis/MaxDepth": "0.0",
            # 0=Features Matching, 1=Optical Flow
            "Vis/CorType": "0",
        }
    ]

    remappings = [
        ("rgb/image", topic("camera/image_raw")),
        ("rgb/camera_info", topic("camera/camera_info")),
        ("depth/image", topic("depth_camera/image_raw")),
        ("depth/camera_info", topic("depth_camera/camera_info")),
        ("imu", topic("imu")),
        ("odom", topic("odom_rgbd")),
    ]

    return LaunchDescription(
        [
            robot_ns_cmd,
            use_sim_time_cmd,
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="log",
                parameters=parameters,
                remappings=remappings,
                arguments=["--ros-args", "--log-level", "Error"],
            ),
        ]
    )
