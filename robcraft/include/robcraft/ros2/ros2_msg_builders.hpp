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

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "robcraft/engine/math/quaternion.hpp"
#include "robcraft/engine/math/vec3.hpp"

namespace robcraft::sensors::lidar {
struct LidarSensor2D;
}

namespace robcraft::sensors::lidar3d {
struct LidarSensor3D;
}

namespace robcraft::sensors::imu {
struct ImuSensor;
}

namespace robcraft::sensors::gps {
struct GpsSensor;
}

namespace robcraft::sensors::magnetometer {
struct MagnetometerSensor;
}

namespace robcraft::sensors::camera {
struct CameraSensor;
}

namespace robcraft::sensors::depth_camera {
struct DepthCameraSensor;
}

namespace robcraft::ros2 {

using namespace robcraft::engine::math;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;

/**
 * @brief Converts simulation seconds to a ROS 2 time message.
 * @param t Time in seconds.
 * @return The equivalent builtin_interfaces Time message.
 */
builtin_interfaces::msg::Time to_ros_time(double t);

/**
 * @brief Fills a TransformStamped's transform from a sim-frame pose, converting to REP-103.
 * @param ts The message whose transform fields are populated (header must already be set).
 * @param sim_pos Sim-frame position to convert.
 * @param sim_rot Sim-frame orientation to convert.
 */
void fill_transform(geometry_msgs::msg::TransformStamped& ts, const Vec3& sim_pos,
                    const Quaternion& sim_rot);

/**
 * @brief Builds a static (zero-stamped) TF between two frames from a sim pose.
 * @param parent Parent frame id.
 * @param child Child frame id.
 * @param sim_pos Sim-frame position to convert.
 * @param sim_rot Sim-frame orientation to convert.
 * @return The populated TransformStamped.
 */
geometry_msgs::msg::TransformStamped make_static_tf(const std::string& parent,
                                                    const std::string& child, const Vec3& sim_pos,
                                                    const Quaternion& sim_rot);

/**
 * @brief Builds a stamped TF between two frames from a sim pose.
 * @param stamp Time stamp for the header.
 * @param parent Parent frame id.
 * @param child Child frame id.
 * @param sim_pos Sim-frame position to convert.
 * @param sim_rot Sim-frame orientation to convert.
 * @return The populated TransformStamped.
 */
geometry_msgs::msg::TransformStamped make_stamped_tf(const builtin_interfaces::msg::Time& stamp,
                                                     const std::string& parent,
                                                     const std::string& child, const Vec3& sim_pos,
                                                     const Quaternion& sim_rot);

/**
 * @brief Builds the fixed static transform from a camera mount frame to its optical frame.
 * The camera mount frame is REP-103 (X forward, Y left, Z up); the optical frame follows the
 * standard camera convention (+Z points out of the lens, +X right, +Y down).
 * @param parent Camera mount frame id (e.g. .../camera_link).
 * @param child Optical frame id (e.g. .../camera_optical_frame).
 * @return The populated zero-stamped TransformStamped with the optical rotation.
 */
geometry_msgs::msg::TransformStamped make_camera_optical_tf(const std::string& parent,
                                                            const std::string& child);

/**
 * @brief Fills a sensor_msgs::msg::Image header and row metadata.
 * @param msg The message whose header and metadata fields are populated.
 * @param stamp Time stamp for the header.
 * @param frame_id Frame id for the header.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @param encoding Encoding string for the image data.
 * @param step Row step in bytes.
 */
void fill_image_header(sensor_msgs::msg::Image& msg, const builtin_interfaces::msg::Time& stamp,
                       const std::string& frame_id, uint32_t width, uint32_t height,
                       const std::string& encoding, uint32_t step);

/**
 * @brief Builds an odometry message from a robot pose.
 * @param stamp Time stamp for the header.
 * @param odom_frame Odometry frame id.
 * @param base_frame Base link frame id.
 * @param sim_pos Sim-frame robot position.
 * @param sim_rot Sim-frame robot orientation.
 * @param linear_vel Forward (REP-103 X) velocity of the base in m/s.
 * @param angular_vel Yaw (REP-103 Z) velocity of the base in rad/s.
 * @param odom_noise_stddev Standard deviation of odometry position noise in meters.
 * @return The populated odometry message.
 */
nav_msgs::msg::Odometry make_odometry(const builtin_interfaces::msg::Time& stamp,
                                      const std::string& odom_frame, const std::string& base_frame,
                                      const Vec3& sim_pos, const Quaternion& sim_rot,
                                      double linear_vel, double angular_vel,
                                      double odom_noise_stddev = 0.0);

/**
 * @brief Builds a laser scan message from a 2D LiDAR sensor.
 * @param stamp Time stamp for the header.
 * @param frame_id LiDAR link frame id.
 * @param lidar The sensor whose config and ranges are published.
 * @return The populated LaserScan message.
 */
sensor_msgs::msg::LaserScan make_laser_scan(const builtin_interfaces::msg::Time& stamp,
                                            const std::string& frame_id,
                                            const LidarSensor2D& lidar);

/**
 * @brief Builds an IMU message from an IMU sensor, converting the sim-frame
 * estimates to REP-103.
 * @param stamp Time stamp for the header.
 * @param frame_id IMU link frame id.
 * @param imu The sensor whose estimates are published.
 * @return The populated Imu message.
 */
sensor_msgs::msg::Imu make_imu(const builtin_interfaces::msg::Time& stamp,
                               const std::string& frame_id, const ImuSensor& imu);

/**
 * @brief Builds a GPS fix message from a GPS sensor.
 * @param stamp Time stamp for the header.
 * @param frame_id GPS link frame id.
 * @param gps The sensor whose lat/lon/alt are published.
 * @return The populated NavSatFix message.
 */
sensor_msgs::msg::NavSatFix make_nav_sat_fix(const builtin_interfaces::msg::Time& stamp,
                                             const std::string& frame_id, const GpsSensor& gps);

/**
 * @brief Builds a magnetometer message from a magnetometer sensor, converting
 * the sim-frame field to REP-103 and microteslas to tesla.
 * @param stamp Time stamp for the header.
 * @param frame_id Magnetometer link frame id.
 * @param mag The sensor whose field reading is published.
 * @return The populated MagneticField message.
 */
sensor_msgs::msg::MagneticField make_magnetic_field(const builtin_interfaces::msg::Time& stamp,
                                                    const std::string& frame_id,
                                                    const MagnetometerSensor& mag);

/**
 * @brief Builds a camera image message from an RGB camera sensor.
 * @param stamp Time stamp for the header.
 * @param frame_id Camera optical frame id.
 * @param cam The sensor whose image data is published.
 * @return The populated Image message.
 */
sensor_msgs::msg::Image make_camera_image(const builtin_interfaces::msg::Time& stamp,
                                          const std::string& frame_id, const CameraSensor& cam);

/**
 * @brief Builds a camera info message from an RGB camera sensor.
 * @param stamp Time stamp for the header.
 * @param frame_id Camera optical frame id.
 * @param cam The sensor whose intrinsics are published.
 * @return The populated CameraInfo message.
 */
sensor_msgs::msg::CameraInfo make_camera_info(const builtin_interfaces::msg::Time& stamp,
                                              const std::string& frame_id, const CameraSensor& cam);

/**
 * @brief Builds a camera info message from a depth camera sensor.
 * @param stamp Time stamp for the header.
 * @param frame_id Depth camera optical frame id.
 * @param depth The sensor whose intrinsics are published.
 * @return The populated CameraInfo message.
 */
sensor_msgs::msg::CameraInfo make_camera_info(const builtin_interfaces::msg::Time& stamp,
                                              const std::string& frame_id,
                                              const DepthCameraSensor& depth);

/**
 * @brief Builds a depth image message from a depth camera sensor.
 * @param stamp Time stamp for the header.
 * @param frame_id Depth camera optical frame id.
 * @param depth The sensor whose depth buffer is published.
 * @return The populated Image message.
 */
sensor_msgs::msg::Image make_depth_image(const builtin_interfaces::msg::Time& stamp,
                                         const std::string& frame_id,
                                         const DepthCameraSensor& depth);

/**
 * @brief Builds a point cloud message from a 3D LiDAR sensor.
 * @param stamp Time stamp for the header.
 * @param frame_id 3D LiDAR link frame id.
 * @param lidar The sensor whose ranges and directions are published.
 * @return The populated PointCloud2 message.
 */
sensor_msgs::msg::PointCloud2 make_point_cloud2(const builtin_interfaces::msg::Time& stamp,
                                                const std::string& frame_id,
                                                const LidarSensor3D& lidar);

}  // namespace robcraft::ros2
