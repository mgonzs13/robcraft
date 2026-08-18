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

#include "robcraft/ros2/ros2_msg_builders.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "robcraft/engine/math/constants.hpp"
#include "robcraft/engine/math/frame_conversion.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

namespace robcraft::ros2 {

using namespace robcraft::engine::math;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;

namespace {
/**
 * @brief Fills a CameraInfo message from pinhole camera parameters.
 * @param stamp Time stamp for the header.
 * @param frame_id Frame id for the header.
 * @param width Image width in pixels.
 * @param height Image height in pixels.
 * @param fov_deg Vertical field of view in degrees.
 * @return The populated CameraInfo message.
 */
sensor_msgs::msg::CameraInfo make_camera_info_pinhole(const builtin_interfaces::msg::Time& stamp,
                                                      const std::string& frame_id, int width,
                                                      int height, double fov_deg) {
    double fov_rad = robcraft::engine::math::deg_to_rad(fov_deg);
    // The sim renders with fov_deg as the VERTICAL field of view and square
    // pixels, so both focal lengths equal (height / 2) / tan(vfov / 2).
    double fy = (height / 2.0) / std::tan(fov_rad / 2.0);
    double fx = fy;
    double cx = width / 2.0;
    double cy = height / 2.0;

    sensor_msgs::msg::CameraInfo msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);

    msg.distortion_model = "plumb_bob";
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

    return msg;
}
}  // namespace

builtin_interfaces::msg::Time to_ros_time(double t) {
    builtin_interfaces::msg::Time msg;
    double sec = std::floor(t);
    msg.sec = static_cast<int32_t>(sec);
    msg.nanosec = static_cast<uint32_t>((t - sec) * 1e9);
    return msg;
}

void fill_transform(geometry_msgs::msg::TransformStamped& ts, const Vec3& sim_pos,
                    const Quaternion& sim_rot) {
    Vec3 pos = sim_to_rep103_position(sim_pos);
    Quaternion rot = sim_to_rep103_orientation(sim_rot);
    ts.transform.translation.x = pos.x;
    ts.transform.translation.y = pos.y;
    ts.transform.translation.z = pos.z;
    ts.transform.rotation.w = rot.w;
    ts.transform.rotation.x = rot.x;
    ts.transform.rotation.y = rot.y;
    ts.transform.rotation.z = rot.z;
}

geometry_msgs::msg::TransformStamped make_static_tf(const std::string& parent,
                                                    const std::string& child, const Vec3& sim_pos,
                                                    const Quaternion& sim_rot) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp.sec = 0;
    ts.header.stamp.nanosec = 0;
    ts.header.frame_id = parent;
    ts.child_frame_id = child;
    fill_transform(ts, sim_pos, sim_rot);
    return ts;
}

geometry_msgs::msg::TransformStamped make_stamped_tf(const builtin_interfaces::msg::Time& stamp,
                                                     const std::string& parent,
                                                     const std::string& child, const Vec3& sim_pos,
                                                     const Quaternion& sim_rot) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = stamp;
    ts.header.frame_id = parent;
    ts.child_frame_id = child;
    fill_transform(ts, sim_pos, sim_rot);
    return ts;
}

geometry_msgs::msg::TransformStamped make_camera_optical_tf(const std::string& parent,
                                                            const std::string& child) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp.sec = 0;
    ts.header.stamp.nanosec = 0;
    ts.header.frame_id = parent;
    ts.child_frame_id = child;

    // Optical frames are expressed in REP-103 (not sim) space: a camera mounted
    // X-forward/Y-left/Z-up must be rotated so that +Z points out of the lens.
    // The rotation (roll +90, pitch -90, yaw 0 in this library's ZYX convention)
    // maps mount +X (forward) -> optical +Z, mount -Y (right) -> optical +X, and
    // mount -Z (down) -> optical +Y. The TF stores the inverse (the optical axes
    // expressed in the mount frame), so the quaternion is conjugated.
    Quaternion optical = Quaternion::from_euler(robcraft::engine::math::kPi / 2.0,
                                                -robcraft::engine::math::kPi / 2.0, 0.0)
                             .conjugate();
    ts.transform.translation.x = 0.0;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.0;
    ts.transform.rotation.w = optical.w;
    ts.transform.rotation.x = optical.x;
    ts.transform.rotation.y = optical.y;
    ts.transform.rotation.z = optical.z;
    return ts;
}

void fill_image_header(sensor_msgs::msg::Image& msg, const builtin_interfaces::msg::Time& stamp,
                       const std::string& frame_id, uint32_t width, uint32_t height,
                       const std::string& encoding, uint32_t step) {
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = height;
    msg.width = width;
    msg.encoding = encoding;
    msg.is_bigendian = 0;
    msg.step = step;
}

nav_msgs::msg::Odometry make_odometry(const builtin_interfaces::msg::Time& stamp,
                                      const std::string& odom_frame, const std::string& base_frame,
                                      const Vec3& sim_pos, const Quaternion& sim_rot,
                                      double linear_vel, double angular_vel,
                                      double odom_noise_stddev) {
    Vec3 pos = sim_to_rep103_position(sim_pos);
    Quaternion rot = sim_to_rep103_orientation(sim_rot);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame;
    msg.child_frame_id = base_frame;

    msg.pose.pose.position.x = pos.x;
    msg.pose.pose.position.y = pos.y;
    msg.pose.pose.position.z = pos.z;

    msg.pose.pose.orientation.w = rot.w;
    msg.pose.pose.orientation.x = rot.x;
    msg.pose.pose.orientation.y = rot.y;
    msg.pose.pose.orientation.z = rot.z;

    // Body-frame twist (REP-103: X-forward, Z-up): the forward velocity from
    // the differential drive and the yaw rate. Downstream filters (e.g.
    // robot_localization EKFs) need this to keep the velocity states bounded
    // between pose updates; without it the estimate drifts while stopped.
    msg.twist.twist.linear.x = linear_vel;
    msg.twist.twist.angular.z = angular_vel;

    const double var = odom_noise_stddev * odom_noise_stddev;
    msg.pose.covariance[0] = var;
    msg.pose.covariance[7] = var;
    msg.pose.covariance[14] = var;

    return msg;
}

sensor_msgs::msg::LaserScan make_laser_scan(const builtin_interfaces::msg::Time& stamp,
                                            const std::string& frame_id,
                                            const LidarSensor2D& lidar) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    const size_t ray_count =
        std::min(static_cast<size_t>(std::max(lidar.num_rays, 0)), lidar.last_ranges.size());
    if (ray_count == 1 && !lidar.last_angles.empty()) {
        msg.angle_min = static_cast<float>(lidar.last_angles[0]);
        msg.angle_max = msg.angle_min;
    } else {
        msg.angle_min = static_cast<float>(lidar.angle_min);
        msg.angle_max = static_cast<float>(lidar.angle_max);
    }
    msg.angle_increment =
        ray_count > 1 ? static_cast<float>((lidar.angle_max - lidar.angle_min) / (ray_count - 1))
                      : 0.0f;
    msg.time_increment = 0.0f;
    msg.scan_time = lidar.update_rate > 0.0 ? static_cast<float>(1.0 / lidar.update_rate) : 0.0f;
    msg.range_min = static_cast<float>(lidar.range_min);
    msg.range_max = static_cast<float>(lidar.range_max);

    msg.ranges.resize(ray_count);
    for (size_t i = 0; i < ray_count; ++i) {
        msg.ranges[i] = static_cast<float>(lidar.last_ranges[i]);
    }

    return msg;
}

sensor_msgs::msg::Imu make_imu(const builtin_interfaces::msg::Time& stamp,
                               const std::string& frame_id, const ImuSensor& imu) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    // The sim computes IMU data in the sim (Y-up/Z-forward) body frame; ROS 2
    // expects REP-103 (X-forward/Y-left/Z-up), so everything is rebased.
    Quaternion orientation = sim_to_rep103_orientation(imu.orientation);
    Vec3 angular_velocity = sim_to_rep103_position(imu.angular_velocity);
    Vec3 linear_acceleration = sim_to_rep103_position(imu.linear_acceleration);

    msg.orientation.w = orientation.w;
    msg.orientation.x = orientation.x;
    msg.orientation.y = orientation.y;
    msg.orientation.z = orientation.z;

    msg.angular_velocity.x = angular_velocity.x;
    msg.angular_velocity.y = angular_velocity.y;
    msg.angular_velocity.z = angular_velocity.z;

    msg.linear_acceleration.x = linear_acceleration.x;
    msg.linear_acceleration.y = linear_acceleration.y;
    msg.linear_acceleration.z = linear_acceleration.z;

    // The sim's orientation is the exact sensor rotation (no added noise), so
    // it is reported with a tiny covariance. The gyro and accelerometer
    // variances are derived from the sensor noise so downstream filters (e.g.
    // an EKF) weight the measurements correctly instead of trusting them
    // infinitely.
    const double ang_var = imu.angular_velocity_noise_stddev * imu.angular_velocity_noise_stddev;
    const double lin_var =
        imu.linear_acceleration_noise_stddev * imu.linear_acceleration_noise_stddev;
    msg.orientation_covariance = {1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6};
    msg.angular_velocity_covariance = {ang_var, 0.0, 0.0, 0.0, ang_var, 0.0, 0.0, 0.0, ang_var};
    msg.linear_acceleration_covariance = {lin_var, 0.0, 0.0, 0.0, lin_var, 0.0, 0.0, 0.0, lin_var};

    return msg;
}

sensor_msgs::msg::NavSatFix make_nav_sat_fix(const builtin_interfaces::msg::Time& stamp,
                                             const std::string& frame_id, const GpsSensor& gps) {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    msg.latitude = gps.latitude;
    msg.longitude = gps.longitude;
    msg.altitude = gps.altitude;

    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    // Report the configured noise as variance so downstream filters (e.g.
    // robot_localization) weight the fix correctly. A zero covariance would
    // make an EKF trust each fix infinitely (gain 1) and snap the estimate to
    // every noisy sample, which reads as drift while the robot is stopped. A
    // floor is also enforced so even a noiseless GPS never carries infinite
    // weight: a single bad sample would otherwise teleport the filter, and it
    // keeps robot_localization's Mahalanobis gating meaningful.
    const double noise_var = gps.position_noise_stddev * gps.position_noise_stddev;
    const double pos_var = std::max(noise_var, 0.25);
    msg.position_covariance = {pos_var, 0.0, 0.0, 0.0, pos_var, 0.0, 0.0, 0.0, pos_var};
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    return msg;
}

sensor_msgs::msg::MagneticField make_magnetic_field(const builtin_interfaces::msg::Time& stamp,
                                                    const std::string& frame_id,
                                                    const MagnetometerSensor& mag) {
    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    // The sim computes the field in the sim (Y-up/Z-forward) body frame; ROS 2
    // expects REP-103 (X-forward/Y-left/Z-up) in tesla, so the reading is
    // rebased and converted from microteslas.
    Vec3 field = sim_to_rep103_position(mag.magnetic_field) * 1e-6;
    msg.magnetic_field.x = field.x;
    msg.magnetic_field.y = field.y;
    msg.magnetic_field.z = field.z;

    // Report the configured noise as variance so downstream filters (e.g.
    // robot_localization) weight the magnetometer correctly.
    const double var = mag.magnetic_field_noise_stddev * mag.magnetic_field_noise_stddev * 1e-12;
    msg.magnetic_field_covariance = {var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var};

    return msg;
}

sensor_msgs::msg::Image make_camera_image(const builtin_interfaces::msg::Time& stamp,
                                          const std::string& frame_id, const CameraSensor& cam) {
    sensor_msgs::msg::Image msg;
    fill_image_header(msg, stamp, frame_id, static_cast<uint32_t>(cam.width),
                      static_cast<uint32_t>(cam.height), "rgb8",
                      static_cast<uint32_t>(cam.width * 3));
    msg.data = cam.image_data;

    return msg;
}

sensor_msgs::msg::CameraInfo make_camera_info(const builtin_interfaces::msg::Time& stamp,
                                              const std::string& frame_id,
                                              const CameraSensor& cam) {
    return make_camera_info_pinhole(stamp, frame_id, cam.width, cam.height, cam.fov_deg);
}

sensor_msgs::msg::CameraInfo make_camera_info(const builtin_interfaces::msg::Time& stamp,
                                              const std::string& frame_id,
                                              const DepthCameraSensor& depth) {
    return make_camera_info_pinhole(stamp, frame_id, depth.width, depth.height, depth.fov_deg);
}

sensor_msgs::msg::Image make_depth_image(const builtin_interfaces::msg::Time& stamp,
                                         const std::string& frame_id,
                                         const DepthCameraSensor& depth) {
    sensor_msgs::msg::Image msg;
    fill_image_header(msg, stamp, frame_id, static_cast<uint32_t>(depth.width),
                      static_cast<uint32_t>(depth.height), "32FC1",
                      static_cast<uint32_t>(depth.width * 4));
    // Depth buffer holds linearized depth in meters; linearization happens in the sim's render pass
    // before publish.
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(depth.depth_data.data());
    msg.data.assign(bytes, bytes + depth.depth_data.size() * sizeof(float));

    return msg;
}

sensor_msgs::msg::PointCloud2 make_point_cloud2(const builtin_interfaces::msg::Time& stamp,
                                                const std::string& frame_id,
                                                const LidarSensor3D& lidar) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(lidar.last_ranges.size());

    sensor_msgs::msg::PointField pf_x;
    pf_x.name = "x";
    pf_x.offset = 0;
    pf_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    pf_x.count = 1;
    sensor_msgs::msg::PointField pf_y = pf_x;
    pf_y.name = "y";
    pf_y.offset = 4;
    sensor_msgs::msg::PointField pf_z = pf_x;
    pf_z.name = "z";
    pf_z.offset = 8;
    msg.fields = {pf_x, pf_y, pf_z};
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = 12 * msg.width;
    msg.is_dense = false;

    msg.data.resize(msg.width * 12);
    for (size_t i = 0; i < lidar.last_ranges.size(); ++i) {
        float xyz[3];
        if (std::isinf(lidar.last_ranges[i])) {
            // No detection: emit NaN coordinates (is_dense=false) instead of an
            // invalid point at infinity.
            xyz[0] = xyz[1] = xyz[2] = std::numeric_limits<float>::quiet_NaN();
        } else {
            // Points in the sensor frame: local direction * range. The
            // lidar3d_link TF frame is REP-103 (X-forward/Z-up), so the sim-local
            // (Y-up/Z-forward) point must be mapped through the same basis change.
            Vec3 p = robcraft::engine::math::sim_to_rep103_position(lidar.last_dirs[i] *
                                                                    lidar.last_ranges[i]);
            xyz[0] = static_cast<float>(p.x);
            xyz[1] = static_cast<float>(p.y);
            xyz[2] = static_cast<float>(p.z);
        }
        std::memcpy(msg.data.data() + i * 12, xyz, sizeof(xyz));
    }

    return msg;
}

}  // namespace robcraft::ros2
