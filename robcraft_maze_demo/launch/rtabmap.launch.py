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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot_ns = LaunchConfiguration("robot_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")

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

    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Whether to launch rtabmapviz",
    )

    def topic(name):
        return PathJoinSubstitution(["/", robot_ns, name])

    def frame(name):
        return PathJoinSubstitution([robot_ns, name])

    parameters = [
        {
            "frame_id": frame("base_footprint"),
            "map_frame_id": "map",
            "odom_frame_id": frame("odom"),
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan": False,
            "approx_sync": True,
            "publish_tf": True,
            "use_sim_time": use_sim_time,
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_imu": 2,
            "qos_odom": 1,
            # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            "RGBD/Enabled": "true",
            "RGBD/OptimizeMaxError": "0.5",
            "RGBD/OptimizeFromGraphEnd": "false",
            "RGBD/CreateOccupancyGrid": "true",
            "RGBD/LoopClosureIdentityGuess": "false",
            "RGBD/LocalBundleOnLoopClosure": "false",
            "VhEp/Enabled": "false",
            "Rtabmap/CreateIntermediateNodes": "false",
            "GFTT/MinDistance": "7.0",
            "GFTT/QualityLevel": "0.001",
            "GFTT/BlockSize": "3",
            "GFTT/UseHarrisDetector": "true",
            "GFTT/K": "0.04",
            "BRIEF/Bytes": "64",
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
            # 1=kNNFlannKdTree
            "Vis/CorNNType": "1",
            # Grid parameters for the /map occupancy grid
            "Grid/Sensor": "1",
            "Grid/DepthDecimation": "4",
            "Grid/RangeMin": "0.0",
            "Grid/RangeMax": "5.0",
            "Grid/MinClusterSize": "10",
            "Grid/MaxGroundAngle": "45",
            "Grid/NormalK": "20",
            "Grid/ClusterRadius": "0.2",
            "Grid/CellSize": "0.1",
            "Grid/FlatObstacleDetected": "false",
            "Grid/RayTracing": "true",
            "Grid/3D": "true",
            "Grid/MapFrameProjection": "true",
            "GridGlobal/UpdateError": "0.01",
            "GridGlobal/MinSize": "100.0",
            "GridGlobal/Eroded": "true",
            "GridGlobal/FloodFillDepth": "16",
        }
    ]

    remappings = [
        ("rgb/image", topic("camera/image_raw")),
        ("rgb/camera_info", topic("camera/camera_info")),
        ("depth/image", topic("depth_camera/image_raw")),
        ("depth/camera_info", topic("depth_camera/camera_info")),
        ("imu", topic("imu")),
        ("odom", topic("odom_filtered")),
        ("goal", "goal_pose"),
    ]

    return LaunchDescription(
        [
            robot_ns_cmd,
            use_sim_time_cmd,
            launch_rtabmapviz_cmd,
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="log",
                parameters=parameters,
                remappings=remappings,
                arguments=["-d", "--ros-args", "--log-level", "Error"],
            ),
            Node(
                condition=IfCondition(launch_rtabmapviz),
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=parameters,
                remappings=remappings,
            ),
        ]
    )
