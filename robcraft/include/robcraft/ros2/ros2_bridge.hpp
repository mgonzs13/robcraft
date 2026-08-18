// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <unordered_map>

#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/core/random.hpp"
#include "robcraft/engine/core/rate_gate.hpp"
#include "robcraft/engine/ecs/transform.hpp"

namespace robcraft::engine::world {
class World;
}  // namespace robcraft::engine::world

namespace robcraft::ros2 {

using namespace robcraft::engine::core;
using namespace robcraft::engine::world;

/** @brief Bridge that mirrors simulated robots to ROS 2 topics. */
class ROS2Bridge {
public:
    /** @brief Constructs the bridge and initializes the ROS 2 node and executor. */
    ROS2Bridge();
    /** @brief Shuts down ROS 2 if still running. */
    ~ROS2Bridge();

    /**
     * @brief Discovers robots in the world and creates their ROS 2 topics.
     * @param world The world to mirror.
     */
    void init(World& world);

    /**
     * @brief Enables or disables publishing the shared world (map) frame.
     * @param on True to publish map -> odom static transforms, false to omit them.
     */
    void set_publish_world_frame(bool on);

    /**
     * @brief Enables or disables publishing the robot odometry transform (odom -> base_footprint).
     * When disabled, only base_footprint -> base_link and the static sensor transforms are
     * published, letting an external odometry source (e.g. robot_localization) own that edge.
     * @param on True to publish odom -> base_footprint, false to omit it.
     */
    void set_publish_odom_tf(bool on);

    /**
     * @brief Returns whether the ROS 2 context is still active.
     * @return True if the context is up; false after rclcpp::shutdown() (e.g. Ctrl+C).
     */
    bool ok() const;

    /** @brief Processes pending ROS 2 messages (spin once). */
    void spin_once();

    /**
     * @brief Records the latest simulation time for message stamps.
     * @param t Simulation time in seconds.
     * @param dt Simulation tick duration in seconds.
     */
    void set_sim_time(double t, double dt);

    /**
     * @brief Returns the mutex guarding the simulated world state.
     * @return The world mutex; the main loop holds it while mutating the world.
     */
    std::mutex& world_mutex();

    /**
     * @brief Launches the background publisher thread.
     * @note The thread stops in the destructor; call once after init().
     */
    void start_publishing();

private:
    /** @brief Background thread body: publishes all topics at a high cadence. */
    void publish_loop();
    /** @brief Per-robot set of ROS 2 publishers and subscriptions. */
    struct RobotBridge {
        /** @brief The entity this bridge entry belongs to. */
        Entity entity;
        /** @brief Topic and TF namespace for this robot. */
        std::string ns;
        /** @brief Shared global map frame name. */
        std::string map_frame;
        /** @brief Odom frame name for this robot. */
        std::string odom_frame;
        /** @brief Base link frame name for this robot. */
        std::string base_frame;
        /** @brief Ground projection frame (base_link projected onto the terrain). */
        std::string base_footprint_frame;
        /** @brief Spawn (initial) pose anchoring the odom frame (REP-105). */
        robcraft::engine::ecs::Transform3D spawn_transform;
        /** @brief LiDAR link frame name for this robot. */
        std::string lidar_frame;
        /** @brief Camera link frame name for this robot. */
        std::string camera_frame;
        /** @brief Camera optical frame name for this robot. */
        std::string camera_optical_frame;
        /** @brief IMU link frame name for this robot. */
        std::string imu_frame;
        /** @brief GPS link frame name for this robot. */
        std::string gps_frame;
        /** @brief Magnetometer link frame name for this robot. */
        std::string mag_frame;
        /** @brief Depth camera link frame name for this robot. */
        std::string depth_frame;
        /** @brief Depth camera optical frame name for this robot. */
        std::string depth_optical_frame;
        /** @brief 3D LiDAR link frame name for this robot. */
        std::string lidar3d_frame;
        /** @brief Subscription for velocity commands. */
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        /** @brief Publisher for odometry messages. */
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
        /** @brief Publisher for laser scan messages. */
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
        /** @brief Publisher for IMU messages. */
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
        /** @brief Publisher for GPS fix messages. */
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        /** @brief Publisher for magnetometer messages. */
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
        /** @brief Publisher for camera image messages. */
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        /** @brief Publisher for camera info messages. */
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
        /** @brief Publisher for depth camera images. */
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
        /** @brief Publisher for depth camera info messages. */
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_pub;
        /** @brief Publisher for 3D LiDAR point clouds. */
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar3d_pub;
        /** @brief Publisher for TF transform messages. */
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;
        /** @brief Publish rate gate for odometry. */
        robcraft::engine::core::RateGate odom_gate;
        /** @brief Publish rate gate for laser scans. */
        robcraft::engine::core::RateGate scan_gate;
        /** @brief Publish rate gate for IMU messages. */
        robcraft::engine::core::RateGate imu_gate;
        /** @brief Publish rate gate for GPS fixes. */
        robcraft::engine::core::RateGate gps_gate;
        /** @brief Publish rate gate for magnetometer messages. */
        robcraft::engine::core::RateGate mag_gate;
        /** @brief Publish rate gate for camera images. */
        robcraft::engine::core::RateGate image_gate;
        /** @brief Publish rate gate for camera info messages. */
        robcraft::engine::core::RateGate camera_info_gate;
        /** @brief Publish rate gate for depth images. */
        robcraft::engine::core::RateGate depth_gate;
        /** @brief Publish rate gate for depth camera info messages. */
        robcraft::engine::core::RateGate depth_camera_info_gate;
        /** @brief Publish rate gate for 3D lidar point clouds. */
        robcraft::engine::core::RateGate lidar3d_gate;
        /** @brief Publish rate gate for dynamic transforms. */
        robcraft::engine::core::RateGate tf_gate;
    };

    /** @brief Creates publishers/subscriptions for one robot under the given namespace. */
    void setup_robot(Entity entity, const std::string& ns);
    /** @brief Applies a received cmd_vel to the robot's drive wheels. */
    void cmd_vel_callback(Entity entity, const geometry_msgs::msg::Twist::SharedPtr msg);
    /** @brief Publishes odometry for a robot. */
    void publish_odom(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes a laser scan for a robot. */
    void publish_scan(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes an IMU message for a robot. */
    void publish_imu(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes a GPS fix for a robot. */
    void publish_gps(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes a magnetometer message for a robot. */
    void publish_magnetometer(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                              double now);
    /** @brief Publishes a camera image for a robot. */
    void publish_image(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes camera intrinsics for a robot. */
    void publish_camera_info(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                             double now);
    /** @brief Publishes a depth camera image for a robot. */
    void publish_depth(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes depth camera intrinsics for a robot. */
    void publish_depth_camera_info(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                                   double now);
    /** @brief Publishes a 3D LiDAR point cloud for a robot. */
    void publish_lidar3d(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes the transform from odom to base_footprint. */
    void publish_tf(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp, double now);
    /** @brief Publishes static transforms (map, sensor mounts) on /tf_static. */
    void publish_static_tfs();
    /** @brief Publishes the simulation clock on /clock. */
    void publish_clock(const builtin_interfaces::msg::Time& stamp);

    /** @brief The ROS 2 node used for all topics. */
    rclcpp::Node::SharedPtr node_;
    /** @brief Publisher for the /clock topic. */
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    /** @brief Publish rate gate for /clock, paced at the sim tick rate. */
    robcraft::engine::core::RateGate clock_gate_;
    /** @brief Publisher for the /tf_static topic. */
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
    /** @brief Single-threaded executor processing callbacks. */
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

    /** @brief Pointer to the simulated world being mirrored. */
    World* world_ = nullptr;
    /** @brief Bridge state keyed by robot entity. */
    std::unordered_map<Entity, RobotBridge> robots_;
    /** @brief Whether the bridge has been initialized. */
    bool initialized_ = false;
    /** @brief Whether to publish the shared world (map) frame static transforms. */
    bool publish_world_frame_ = false;
    /** @brief Whether to publish the robot odometry transform (odom -> base_footprint). */
    bool publish_odom_tf_ = true;
    /** @brief Background thread publishing all topics at a high cadence. */
    std::thread publish_thread_;
    /** @brief Whether the publisher thread should keep running. */
    std::atomic<bool> publish_running_{false};
    /** @brief Guards simulated world state read by the publisher thread. */
    std::mutex world_mutex_;
    /** @brief Latest simulation time for message stamps. */
    double sim_time_ = 0.0;
    /** @brief Simulation tick duration for pacing /clock. */
    double sim_dt_ = 0.01;
    /** @brief Random source for odometry pose noise (fixed seed for reproducibility). */
    robcraft::engine::core::Random rng_{1234};
};

}  // namespace robcraft::ros2
