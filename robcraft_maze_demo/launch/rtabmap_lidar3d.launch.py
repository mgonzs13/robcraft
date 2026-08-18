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
            "frame_id": frame("base_footprint"),
            "map_frame_id": "world",
            "subscribe_odom": True,
            "subscribe_scan_cloud": True,
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_scan": False,
            "publish_tf": False,
            "use_sim_time": use_sim_time,
            "approx_sync": True,
            "qos_odom": 1,
            "qos_scan": 1,
            # Hypotheses selection
            "Rtabmap/LoopGPS": "false",
            "Rtabmap/LoopThr": "0.11",
            # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            "RGBD/Enabled": "true",
            "Rtabmap/CreateIntermediateNodes": "false",
            # 0=Vis, 1=Icp, 2=VisIcp
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "true",
            # ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).
            "Icp/Strategy": "1",
            "Icp/MaxTranslation": "0.2",
            "Icp/VoxelSize": "0.05",
            "Icp/DownsamplingStep": "1",
            "Icp/MaxCorrespondenceDistance": "0.1",
            "Icp/Iterations": "30",
            "Icp/Epsilon": "0.0",
            "Icp/CorrespondenceRatio": "0.1",
            "Icp/PointToPlane": "true",
            # Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s).
            "Grid/Sensor": "0",
            "Grid/DepthDecimation": "4",
            "Grid/RangeMin": "0.0",
            "Grid/RangeMax": "10.0",
            "Grid/MinClusterSize": "10",
            "Grid/MaxGroundAngle": "60",
            "Grid/NormalsSegmentation": "true",
            "Grid/NormalK": "20",
            "Grid/ClusterRadius": "0.2",
            "Grid/CellSize": "0.1",
            "Grid/FlatObstacleDetected": "false",
            "Grid/RayTracing": "true",
            "Grid/3D": "true",
            "Grid/MapFrameProjection": "true",
            "Grid/MaxGroundHeight": "0.1",
            "Grid/MaxObstacleHeight": "1.0",
            "GridGlobal/FootprintRadius": "0.4",
            "GridGlobal/UpdateError": "0.01",
            "GridGlobal/MinSize": "300.0",
            "GridGlobal/Eroded": "true",
            "GridGlobal/FloodFillDepth": "16",
        }
    ]

    remappings = [
        ("scan_cloud", topic("lidar3d/points")),
        ("odom", topic("odom_filtered_gps")),
        ("gps/fix", topic("gps")),
        ("imu", topic("imu")),
        ("goal", "goal_pose"),
    ]

    return LaunchDescription(
        [
            robot_ns_cmd,
            use_sim_time_cmd,
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="log",
                parameters=parameters,
                remappings=remappings,
                arguments=["-d", "--ros-args", "--log-level", "Error"],
            ),
        ]
    )
