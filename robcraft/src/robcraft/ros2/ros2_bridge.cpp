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

#include "robcraft/ros2/ros2_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include "robcraft/engine/core/logging.hpp"
#include "robcraft/engine/core/name_utils.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/simulation/simulation_clock.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/ros2/ros2_msg_builders.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

namespace robcraft::ros2 {

using namespace robcraft::engine::core;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::math;
using namespace robcraft::engine::simulation;
using namespace robcraft::engine::world;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;
using namespace robcraft::robots::differential_drive;

namespace {
/**
 * @brief Returns the sensor component when its publisher exists and its rate gate is due.
 * @tparam T Sensor component type.
 * @tparam Pub Publisher shared-pointer type.
 * @param world The simulated world.
 * @param entity The robot entity carrying the sensor.
 * @param pub The publisher for the topic (may be null when the sensor is absent).
 * @param gate The rate gate pacing this topic.
 * @param now Current wall-clock time in seconds.
 * @return The sensor component, or null when the publisher is missing or the gate denies.
 */
template <typename T, typename Pub>
T* sensor_due(World& world, Entity entity, const Pub& pub, RateGate& gate, double now) {
    if (!pub) return nullptr;
    auto* sensor = world.get_component<T>(entity);
    if (!sensor) return nullptr;
    if (!gate.due(now, 1.0 / sensor->update_rate)) return nullptr;
    return sensor;
}

/**
 * @brief Returns a component when its publisher exists and its rate gate is due.
 * @tparam T Component type.
 * @tparam Pub Publisher shared-pointer type.
 * @param world The simulated world.
 * @param entity The robot entity carrying the component.
 * @param pub The publisher for the topic (may be null).
 * @param gate The rate gate pacing this topic.
 * @param now Current wall-clock time in seconds.
 * @param rate Publish rate in Hz used to size the gate interval.
 * @return The component, or null when the publisher is missing or the gate denies.
 */
template <typename T, typename Pub>
T* component_due(World& world, Entity entity, const Pub& pub, RateGate& gate, double now,
                 double rate) {
    if (!pub) return nullptr;
    auto* comp = world.get_component<T>(entity);
    if (!comp) return nullptr;
    if (!gate.due(now, 1.0 / rate)) return nullptr;
    return comp;
}

/**
 * @brief Creates a reliable publisher with a default queue depth.
 * @tparam T Message type.
 * @param node The ROS 2 node owning the publisher.
 * @param topic Fully-qualified topic name.
 * @return The created publisher.
 */
template <typename T>
typename rclcpp::Publisher<T>::SharedPtr make_pub(rclcpp::Node& node, const std::string& topic) {
    return node.create_publisher<T>(topic, rclcpp::QoS(10));
}

/**
 * @brief Returns the odometry publish rate for a robot, falling back to 30 Hz.
 * @param drive The robot's drive component (may be null).
 * @return The effective odometry rate in Hz.
 */
double odom_rate(const DifferentialDrive* drive) {
    return drive ? drive->odom_rate : 30.0;
}

/**
 * @brief Expresses a world pose relative to a reference (spawn) pose.
 *
 * The odom frame is anchored at the robot's spawn pose (REP-105), so every
 * pose published in the odom frame is the world pose composed with the inverse
 * of the spawn pose. At the start of the simulation this yields the identity.
 * @param spawn The reference (spawn) pose.
 * @param pos The world position to relativize.
 * @param rot The world rotation to relativize.
 * @return The position and rotation expressed in the odom frame.
 */
std::pair<Vec3, Quaternion> pose_relative_to(const Transform3D& spawn, const Vec3& pos,
                                             const Quaternion& rot) {
    Quaternion inv = spawn.rotation.conjugate();
    return {inv.rotate(pos - spawn.position), inv * rot};
}
}  // namespace

ROS2Bridge::ROS2Bridge() {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    this->node_ = std::make_shared<rclcpp::Node>("robcraft", opts);

    this->clock_pub_ =
        this->node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));

    rclcpp::PublisherOptions tf_static_options;
    tf_static_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    this->tf_static_pub_ = this->node_->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).transient_local(), tf_static_options);

    this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->executor_->add_node(this->node_);
}

ROS2Bridge::~ROS2Bridge() {
    this->publish_running_ = false;
    if (this->publish_thread_.joinable()) this->publish_thread_.join();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

bool ROS2Bridge::ok() const {
    return rclcpp::ok();
}

void ROS2Bridge::set_publish_world_frame(bool on) {
    if (this->publish_world_frame_ == on) return;
    this->publish_world_frame_ = on;
    if (this->initialized_) this->publish_static_tfs();
}

void ROS2Bridge::set_publish_odom_tf(bool on) {
    this->publish_odom_tf_ = on;
}

void ROS2Bridge::init(World& world) {
    this->world_ = &world;
    this->robots_.clear();

    auto* dd_store = world.store<DifferentialDrive>();
    if (!dd_store) return;

    std::vector<Entity> robots;
    robots.reserve(dd_store->size());
    for (auto& [entity, dd] : *dd_store) robots.push_back(entity);
    std::sort(robots.begin(), robots.end());

    std::unordered_map<std::string, int> base_counts;
    for (Entity entity : robots) {
        auto* name = this->world_->get_component<Name>(entity);
        std::string raw = name ? name->value : "";
        int index = ++base_counts[robot_base_name(raw, entity)];
        this->setup_robot(entity, robot_namespace(raw, entity, index));
    }

    this->publish_static_tfs();

    auto log = get_logger();
    log->info("ROS2Bridge: {} robots", this->robots_.size());

    this->initialized_ = true;
}

void ROS2Bridge::setup_robot(Entity entity, const std::string& ns) {
    RobotBridge rb;
    rb.entity = entity;

    rb.ns = ns;
    rb.map_frame = "world";
    rb.odom_frame = ns + "/odom";
    rb.base_frame = ns + "/base_link";
    rb.base_footprint_frame = ns + "/base_footprint";
    rb.lidar_frame = ns + "/lidar_link";
    rb.camera_frame = ns + "/camera_link";
    rb.camera_optical_frame = ns + "/camera_optical_frame";
    rb.imu_frame = ns + "/imu_link";
    rb.gps_frame = ns + "/gps_link";
    rb.mag_frame = ns + "/mag_link";
    rb.depth_frame = ns + "/depth_link";
    rb.depth_optical_frame = ns + "/depth_optical_frame";
    rb.lidar3d_frame = ns + "/lidar3d_link";

    // The odom frame is anchored at the robot's spawn pose (REP-105): odom ->
    // base starts at the identity and world -> odom carries the spawn offset.
    if (auto* spawn = this->world_->get_component<Transform3D>(entity)) {
        rb.spawn_transform = *spawn;
    } else {
        rb.spawn_transform = Transform3D{};
    }

    rb.cmd_vel_sub = this->node_->create_subscription<geometry_msgs::msg::Twist>(
        "/" + ns + "/cmd_vel", rclcpp::QoS(10),
        [this, entity](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->cmd_vel_callback(entity, msg);
        });

    rb.odom_pub = make_pub<nav_msgs::msg::Odometry>(*this->node_, "/" + ns + "/odom");

    if (this->world_->get_component<LidarSensor2D>(entity)) {
        rb.scan_pub = make_pub<sensor_msgs::msg::LaserScan>(*this->node_, "/" + ns + "/scan");
    }

    if (this->world_->get_component<ImuSensor>(entity)) {
        rb.imu_pub = make_pub<sensor_msgs::msg::Imu>(*this->node_, "/" + ns + "/imu");
    }

    if (this->world_->get_component<GpsSensor>(entity)) {
        rb.gps_pub = make_pub<sensor_msgs::msg::NavSatFix>(*this->node_, "/" + ns + "/gps");
    }

    if (this->world_->get_component<MagnetometerSensor>(entity)) {
        rb.mag_pub = make_pub<sensor_msgs::msg::MagneticField>(*this->node_, "/" + ns + "/mag");
    }

    if (this->world_->get_component<CameraSensor>(entity)) {
        rb.image_pub =
            make_pub<sensor_msgs::msg::Image>(*this->node_, "/" + ns + "/camera/image_raw");
        rb.camera_info_pub =
            make_pub<sensor_msgs::msg::CameraInfo>(*this->node_, "/" + ns + "/camera/camera_info");
    }

    if (this->world_->get_component<DepthCameraSensor>(entity)) {
        rb.depth_pub =
            make_pub<sensor_msgs::msg::Image>(*this->node_, "/" + ns + "/depth_camera/image_raw");
        rb.depth_camera_info_pub = make_pub<sensor_msgs::msg::CameraInfo>(
            *this->node_, "/" + ns + "/depth_camera/camera_info");
    }

    if (this->world_->get_component<LidarSensor3D>(entity)) {
        rb.lidar3d_pub =
            make_pub<sensor_msgs::msg::PointCloud2>(*this->node_, "/" + ns + "/lidar3d/points");
    }

    rb.tf_pub = make_pub<tf2_msgs::msg::TFMessage>(*this->node_, "/tf");

    this->robots_[entity] = std::move(rb);
}

void ROS2Bridge::cmd_vel_callback(Entity entity, const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!this->world_) return;

    auto* dd = this->world_->get_component<DifferentialDrive>(entity);
    if (!dd) return;

    double v = msg->linear.x;
    double w = msg->angular.z;
    double half_wb = dd->wheel_base * 0.5;

    dd->left_velocity = v - w * half_wb;
    dd->right_velocity = v + w * half_wb;
}

void ROS2Bridge::spin_once() {
    if (this->executor_ && rclcpp::ok()) {
        this->executor_->spin_some();
    }
}

void ROS2Bridge::publish_loop() {
    while (this->publish_running_) {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        if (!this->initialized_ || !this->world_ || !rclcpp::ok()) continue;

        std::lock_guard<std::mutex> lk(this->world_mutex_);
        // rclcpp::ok() can flip while we were waiting on the lock (Ctrl+C during
        // a world reload); re-check before publishing.
        if (!rclcpp::ok()) continue;

        try {
            double now =
                std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                    .count();

            auto stamp = to_ros_time(this->sim_time_);
            // /clock is paced at the sim tick rate so a fast loop doesn't flood it.
            if (this->clock_gate_.due(now, this->sim_dt_)) this->publish_clock(stamp);

            for (auto& [entity, rb] : this->robots_) {
                // TF first so consumers (e.g. RTAB-Map) find the transform at a
                // sensor's exact stamp before processing that sensor's message.
                this->publish_tf(rb, stamp, now);
                this->publish_odom(rb, stamp, now);
                this->publish_scan(rb, stamp, now);
                this->publish_imu(rb, stamp, now);
                this->publish_gps(rb, stamp, now);
                this->publish_magnetometer(rb, stamp, now);
                this->publish_image(rb, stamp, now);
                this->publish_camera_info(rb, stamp, now);
                this->publish_depth(rb, stamp, now);
                this->publish_depth_camera_info(rb, stamp, now);
                this->publish_lidar3d(rb, stamp, now);
            }
        } catch (const std::exception&) {
            // A shutdown can land mid-publish (rclcpp::ok() flips asynchronously);
            // stop cleanly instead of std::terminate on the publisher thread.
            if (!rclcpp::ok()) break;
        }
    }
}

void ROS2Bridge::set_sim_time(double t, double dt) {
    std::lock_guard<std::mutex> lk(this->world_mutex_);
    this->sim_time_ = t;
    this->sim_dt_ = dt;
}

std::mutex& ROS2Bridge::world_mutex() {
    return this->world_mutex_;
}

void ROS2Bridge::start_publishing() {
    this->publish_running_ = true;
    this->publish_thread_ = std::thread([this]() { this->publish_loop(); });
}

void ROS2Bridge::publish_clock(const builtin_interfaces::msg::Time& stamp) {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = stamp;
    this->clock_pub_->publish(msg);
}

void ROS2Bridge::publish_odom(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                              double now) {
    auto* drive = this->world_->get_component<DifferentialDrive>(rb.entity);
    auto* tf = component_due<Transform3D>(*this->world_, rb.entity, rb.odom_pub, rb.odom_gate, now,
                                          odom_rate(drive));
    if (!tf) return;

    auto [rel_pos, rel_rot] = pose_relative_to(rb.spawn_transform, tf->position, tf->rotation);
    if (drive->odom_noise_stddev > 0.0) {
        rel_pos.x += this->rng_.gaussian(0.0, drive->odom_noise_stddev);
        rel_pos.y += this->rng_.gaussian(0.0, drive->odom_noise_stddev);
        rel_pos.z += this->rng_.gaussian(0.0, drive->odom_noise_stddev);
    }
    rb.odom_pub->publish(make_odometry(stamp, rb.odom_frame, rb.base_frame, rel_pos, rel_rot,
                                       drive->linear_velocity(), drive->angular_velocity(),
                                       drive->odom_noise_stddev));
}

void ROS2Bridge::publish_scan(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                              double now) {
    auto* lidar =
        sensor_due<LidarSensor2D>(*this->world_, rb.entity, rb.scan_pub, rb.scan_gate, now);
    if (!lidar) return;
    if (lidar->last_ranges.size() != static_cast<size_t>(lidar->num_rays)) return;

    rb.scan_pub->publish(make_laser_scan(stamp, rb.lidar_frame, *lidar));
}

void ROS2Bridge::publish_imu(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                             double now) {
    auto* imu = sensor_due<ImuSensor>(*this->world_, rb.entity, rb.imu_pub, rb.imu_gate, now);
    if (!imu) return;

    rb.imu_pub->publish(make_imu(stamp, rb.imu_frame, *imu));
}

void ROS2Bridge::publish_gps(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                             double now) {
    auto* gps = sensor_due<GpsSensor>(*this->world_, rb.entity, rb.gps_pub, rb.gps_gate, now);
    if (!gps) return;

    rb.gps_pub->publish(make_nav_sat_fix(stamp, rb.gps_frame, *gps));
}

void ROS2Bridge::publish_magnetometer(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                                      double now) {
    auto* mag =
        sensor_due<MagnetometerSensor>(*this->world_, rb.entity, rb.mag_pub, rb.mag_gate, now);
    if (!mag) return;

    rb.mag_pub->publish(make_magnetic_field(stamp, rb.mag_frame, *mag));
}

void ROS2Bridge::publish_tf(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                            double now) {
    auto* drive = this->world_->get_component<DifferentialDrive>(rb.entity);
    auto* tf = component_due<Transform3D>(*this->world_, rb.entity, rb.tf_pub, rb.tf_gate, now,
                                          odom_rate(drive));
    if (!tf) return;

    // base_footprint sits on the terrain directly under base_link; base_link
    // floats above it by the robot's ground clearance (the sim keeps base_link
    // at ground + ground_frac * scale.y, so the offset is constant).
    float ground_y = this->world_->terrain().height_at_world(tf->position.x, tf->position.z);
    Vec3 footprint(tf->position.x, ground_y, tf->position.z);
    float clearance = static_cast<float>(tf->position.y - ground_y);

    tf2_msgs::msg::TFMessage msg;

    if (this->publish_odom_tf_) {
        auto [rel_footprint, rel_rot] =
            pose_relative_to(rb.spawn_transform, footprint, tf->rotation);
        msg.transforms.push_back(
            make_stamped_tf(stamp, rb.odom_frame, rb.base_footprint_frame, rel_footprint, rel_rot));
    }
    msg.transforms.push_back(make_stamped_tf(stamp, rb.base_footprint_frame, rb.base_frame,
                                             Vec3(0.0, clearance, 0.0), Quaternion::identity()));

    rb.tf_pub->publish(msg);
}

void ROS2Bridge::publish_static_tfs() {
    tf2_msgs::msg::TFMessage msg;

    for (auto& [entity, rb] : this->robots_) {
        if (this->publish_world_frame_) {
            msg.transforms.push_back(make_static_tf(rb.map_frame, rb.odom_frame,
                                                    rb.spawn_transform.position,
                                                    rb.spawn_transform.rotation));
        }

        auto add_sensor_tf = [&](const std::string& child, const Vec3& pos, const Vec3& euler) {
            msg.transforms.push_back(make_static_tf(
                rb.base_frame, child, pos, Quaternion::from_euler(euler.x, euler.y, euler.z)));
        };

        if (auto* lidar = this->world_->get_component<LidarSensor2D>(entity))
            add_sensor_tf(rb.lidar_frame, lidar->position, lidar->rotation);
        if (auto* cam = this->world_->get_component<CameraSensor>(entity)) {
            add_sensor_tf(rb.camera_frame, cam->position, cam->rotation);
            msg.transforms.push_back(
                make_camera_optical_tf(rb.camera_frame, rb.camera_optical_frame));
        }
        if (auto* imu = this->world_->get_component<ImuSensor>(entity))
            add_sensor_tf(rb.imu_frame, imu->position, imu->rotation);
        if (auto* gps = this->world_->get_component<GpsSensor>(entity))
            add_sensor_tf(rb.gps_frame, gps->position, gps->rotation);
        if (auto* mag = this->world_->get_component<MagnetometerSensor>(entity))
            add_sensor_tf(rb.mag_frame, mag->position, mag->rotation);
        if (auto* depth = this->world_->get_component<DepthCameraSensor>(entity)) {
            add_sensor_tf(rb.depth_frame, depth->position, depth->rotation);
            msg.transforms.push_back(
                make_camera_optical_tf(rb.depth_frame, rb.depth_optical_frame));
        }
        if (auto* lidar3d = this->world_->get_component<LidarSensor3D>(entity))
            add_sensor_tf(rb.lidar3d_frame, lidar3d->position, lidar3d->rotation);
    }

    this->tf_static_pub_->publish(msg);
}

void ROS2Bridge::publish_image(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                               double now) {
    auto* cam =
        sensor_due<CameraSensor>(*this->world_, rb.entity, rb.image_pub, rb.image_gate, now);
    if (!cam) return;
    if (cam->image_data.empty()) return;

    rb.image_pub->publish(make_camera_image(stamp, rb.camera_optical_frame, *cam));
}

void ROS2Bridge::publish_camera_info(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                                     double now) {
    auto* cam = sensor_due<CameraSensor>(*this->world_, rb.entity, rb.camera_info_pub,
                                         rb.camera_info_gate, now);
    if (!cam) return;

    rb.camera_info_pub->publish(make_camera_info(stamp, rb.camera_optical_frame, *cam));
}

void ROS2Bridge::publish_depth(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                               double now) {
    auto* depth =
        sensor_due<DepthCameraSensor>(*this->world_, rb.entity, rb.depth_pub, rb.depth_gate, now);
    if (!depth) return;
    if (depth->depth_data.empty()) return;

    rb.depth_pub->publish(make_depth_image(stamp, rb.depth_optical_frame, *depth));
}

void ROS2Bridge::publish_depth_camera_info(RobotBridge& rb,
                                           const builtin_interfaces::msg::Time& stamp, double now) {
    auto* depth = sensor_due<DepthCameraSensor>(*this->world_, rb.entity, rb.depth_camera_info_pub,
                                                rb.depth_camera_info_gate, now);
    if (!depth) return;

    rb.depth_camera_info_pub->publish(make_camera_info(stamp, rb.depth_optical_frame, *depth));
}

void ROS2Bridge::publish_lidar3d(RobotBridge& rb, const builtin_interfaces::msg::Time& stamp,
                                 double now) {
    auto* lidar3d =
        sensor_due<LidarSensor3D>(*this->world_, rb.entity, rb.lidar3d_pub, rb.lidar3d_gate, now);
    if (!lidar3d) return;

    rb.lidar3d_pub->publish(make_point_cloud2(stamp, rb.lidar3d_frame, *lidar3d));
}

}  // namespace robcraft::ros2
