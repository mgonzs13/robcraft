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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "robcraft/engine/core/entity.hpp"
#include "robcraft/engine/ecs/name.hpp"
#include "robcraft/engine/ecs/transform.hpp"
#include "robcraft/engine/math/frame_conversion.hpp"
#include "robcraft/engine/world/world.hpp"
#include "robcraft/robots/differential_drive/differential_drive.hpp"
#include "robcraft/ros2/ros2_bridge.hpp"
#include "robcraft/ros2/ros2_msg_builders.hpp"
#include "robcraft/sensors/camera/camera_sensor.hpp"
#include "robcraft/sensors/depth_camera/depth_camera_sensor.hpp"
#include "robcraft/sensors/gps/gps_sensor.hpp"
#include "robcraft/sensors/imu/imu_sensor.hpp"
#include "robcraft/sensors/lidar/lidar_sensor.hpp"
#include "robcraft/sensors/lidar3d/lidar3d_sensor.hpp"
#include "robcraft/sensors/magnetometer/magnetometer_sensor.hpp"

using namespace robcraft::engine::core;
using namespace robcraft::engine::ecs;
using namespace robcraft::engine::math;
using namespace robcraft::engine::world;
using namespace robcraft::ros2;
using namespace robcraft::robots::differential_drive;
using namespace robcraft::sensors::camera;
using namespace robcraft::sensors::depth_camera;
using namespace robcraft::sensors::gps;
using namespace robcraft::sensors::imu;
using namespace robcraft::sensors::lidar;
using namespace robcraft::sensors::lidar3d;
using namespace robcraft::sensors::magnetometer;

using Catch::Approx;

TEST_CASE("to_ros_time splits seconds and nanoseconds", "[ros2]") {
    auto t0 = to_ros_time(0.0);
    REQUIRE(t0.sec == 0);
    REQUIRE(t0.nanosec == 0);

    auto t1 = to_ros_time(1.5);
    REQUIRE(t1.sec == 1);
    REQUIRE(t1.nanosec == 500000000);

    auto t2 = to_ros_time(-0.25);
    REQUIRE(t2.sec == -1);
    REQUIRE(t2.nanosec == 750000000);
}

TEST_CASE("make_static_tf converts sim pose to REP-103", "[ros2]") {
    auto ts = make_static_tf("world", "robot_1/odom", Vec3(1.0, 2.0, 3.0), Quaternion::identity());

    REQUIRE(ts.header.stamp.sec == 0);
    REQUIRE(ts.header.stamp.nanosec == 0);
    REQUIRE(ts.header.frame_id == "world");
    REQUIRE(ts.child_frame_id == "robot_1/odom");

    // sim (Y-up, Z-forward) -> REP-103 (X-forward, Z-up): (x,y,z) -> (z,x,y).
    REQUIRE(ts.transform.translation.x == Approx(3.0));
    REQUIRE(ts.transform.translation.y == Approx(1.0));
    REQUIRE(ts.transform.translation.z == Approx(2.0));
    REQUIRE(ts.transform.rotation.w == Approx(1.0));
}

TEST_CASE("make_stamped_tf preserves the stamp", "[ros2]") {
    auto stamp = to_ros_time(42.25);
    auto ts = make_stamped_tf(stamp, "odom", "base_link", Vec3{}, Quaternion::identity());

    REQUIRE(ts.header.stamp == stamp);
    REQUIRE(ts.header.frame_id == "odom");
    REQUIRE(ts.child_frame_id == "base_link");
}

TEST_CASE("make_odometry fills frames and REP-103 pose", "[ros2]") {
    auto stamp = to_ros_time(1.0);
    auto odom = make_odometry(stamp, "robot_1/odom", "robot_1/base_link", Vec3(10.0, 0.0, 5.0),
                              Quaternion::identity(), 0.5, -0.25);

    REQUIRE(odom.header.stamp == stamp);
    REQUIRE(odom.header.frame_id == "robot_1/odom");
    REQUIRE(odom.child_frame_id == "robot_1/base_link");
    REQUIRE(odom.pose.pose.position.x == Approx(5.0));
    REQUIRE(odom.pose.pose.position.y == Approx(10.0));
    REQUIRE(odom.pose.pose.position.z == Approx(0.0));
    REQUIRE(odom.pose.pose.orientation.w == Approx(1.0));
    REQUIRE(odom.twist.twist.linear.x == Approx(0.5));
    REQUIRE(odom.twist.twist.angular.z == Approx(-0.25));
}

TEST_CASE("make_odometry fills pose covariance from odom noise", "[ros2]") {
    auto stamp = to_ros_time(1.0);

    auto odom = make_odometry(stamp, "robot_1/odom", "robot_1/base_link", Vec3(10.0, 0.0, 5.0),
                              Quaternion::identity(), 0.5, -0.25, 0.5);
    REQUIRE(odom.pose.covariance[0] == Approx(0.25));
    REQUIRE(odom.pose.covariance[7] == Approx(0.25));
    REQUIRE(odom.pose.covariance[14] == Approx(0.25));
    REQUIRE(odom.pose.covariance[1] == Approx(0.0));

    auto clean = make_odometry(stamp, "robot_1/odom", "robot_1/base_link", Vec3(0.0, 0.0, 0.0),
                               Quaternion::identity(), 0.0, 0.0, 0.0);
    REQUIRE(clean.pose.covariance[0] == Approx(0.0));
    REQUIRE(clean.pose.covariance[7] == Approx(0.0));
    REQUIRE(clean.pose.covariance[14] == Approx(0.0));
}

TEST_CASE("make_laser_scan copies sensor config and ranges", "[ros2]") {
    LidarSensor2D lidar;
    lidar.num_rays = 3;
    lidar.rebuild_angles();
    lidar.last_ranges[0] = 0.5;
    lidar.last_ranges[1] = 1.5;
    lidar.last_ranges[2] = std::numeric_limits<double>::infinity();

    auto scan = make_laser_scan(to_ros_time(2.0), "robot_1/lidar_link", lidar);

    REQUIRE(scan.header.frame_id == "robot_1/lidar_link");
    REQUIRE(scan.range_min == Approx(0.05));
    REQUIRE(scan.range_max == Approx(20.0));
    REQUIRE(scan.angle_min == Approx(lidar.angle_min));
    REQUIRE(scan.angle_max == Approx(lidar.angle_max));
    REQUIRE(scan.scan_time == Approx(1.0 / 15.0));
    REQUIRE(scan.ranges.size() == 3);
    REQUIRE(scan.ranges[0] == Approx(0.5));
    REQUIRE(scan.ranges[1] == Approx(1.5));
    REQUIRE(std::isinf(scan.ranges[2]));
}

TEST_CASE("make_laser_scan handles a single ray", "[ros2]") {
    LidarSensor2D lidar;
    lidar.num_rays = 1;
    lidar.angle_min = -0.25;
    lidar.angle_max = 0.25;
    lidar.rebuild_angles();

    auto scan = make_laser_scan(to_ros_time(2.0), "robot_1/lidar_link", lidar);

    REQUIRE(scan.ranges.size() == 1);
    REQUIRE(scan.angle_min == Approx(0.0));
    REQUIRE(scan.angle_max == Approx(0.0));
    REQUIRE(scan.angle_increment == Approx(0.0));
    REQUIRE(scan.scan_time == Approx(1.0 / lidar.update_rate));
}

TEST_CASE("make_laser_scan rejects a negative ray count", "[ros2]") {
    LidarSensor2D lidar;
    lidar.num_rays = -1;

    auto scan = make_laser_scan(to_ros_time(2.0), "robot_1/lidar_link", lidar);

    REQUIRE(scan.ranges.empty());
}

TEST_CASE("make_imu converts sim-frame estimates to REP-103", "[ros2]") {
    ImuSensor imu;
    imu.orientation = Quaternion::from_euler(0.1, 0.2, 0.3);
    imu.angular_velocity = Vec3(1.0, 2.0, 3.0);
    imu.linear_acceleration = Vec3(4.0, 5.0, 6.0);

    auto msg = make_imu(to_ros_time(3.0), "robot_1/imu_link", imu);

    REQUIRE(msg.header.frame_id == "robot_1/imu_link");
    // The sim (Y-up, Z-forward) basis maps to REP-103 (X-forward, Z-up) as
    // (x, y, z) -> (z, x, y), so vectors are permuted accordingly.
    REQUIRE(msg.angular_velocity.x == Approx(3.0));
    REQUIRE(msg.angular_velocity.y == Approx(1.0));
    REQUIRE(msg.angular_velocity.z == Approx(2.0));
    REQUIRE(msg.linear_acceleration.x == Approx(6.0));
    REQUIRE(msg.linear_acceleration.y == Approx(4.0));
    REQUIRE(msg.linear_acceleration.z == Approx(5.0));

    Quaternion expected = sim_to_rep103_orientation(Quaternion::from_euler(0.1, 0.2, 0.3));
    REQUIRE(msg.orientation.w == Approx(expected.w));
    REQUIRE(msg.orientation.x == Approx(expected.x));
    REQUIRE(msg.orientation.y == Approx(expected.y));
    REQUIRE(msg.orientation.z == Approx(expected.z));
}

TEST_CASE("make_nav_sat_fix copies lat/lon/alt and fix status", "[ros2]") {
    GpsSensor gps;
    gps.latitude = 40.0;
    gps.longitude = -3.0;
    gps.altitude = 10.0;

    auto msg = make_nav_sat_fix(to_ros_time(4.0), "robot_1/gps_link", gps);

    REQUIRE(msg.header.frame_id == "robot_1/gps_link");
    REQUIRE(msg.latitude == Approx(40.0));
    REQUIRE(msg.longitude == Approx(-3.0));
    REQUIRE(msg.altitude == Approx(10.0));
    REQUIRE(msg.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    REQUIRE(msg.status.service == sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
}

TEST_CASE("make_nav_sat_fix enforces a minimum position covariance", "[ros2]") {
    // A noiseless GPS (stddev 0) must still report a non-zero covariance so a
    // downstream EKF does not trust every fix infinitely (which would snap the
    // estimate to any bad sample and appear as a teleport).
    GpsSensor gps;
    gps.position_noise_stddev = 0.0;

    auto msg = make_nav_sat_fix(to_ros_time(4.0), "robot_1/gps_link", gps);

    REQUIRE(msg.position_covariance[0] == Approx(0.25));
    REQUIRE(msg.position_covariance[4] == Approx(0.25));
    REQUIRE(msg.position_covariance[8] == Approx(0.25));
    REQUIRE(msg.position_covariance_type ==
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED);

    // A noisy GPS still reports its configured variance when above the floor.
    GpsSensor noisy;
    noisy.position_noise_stddev = 2.0;
    auto msg2 = make_nav_sat_fix(to_ros_time(4.0), "robot_1/gps_link", noisy);
    REQUIRE(msg2.position_covariance[0] == Approx(4.0));
}

TEST_CASE("make_magnetic_field converts to REP-103 tesla with covariance", "[ros2]") {
    MagnetometerSensor mag;
    mag.magnetic_field = Vec3(0.0, -45.315389, 21.130913);  // uT, sim body frame
    mag.magnetic_field_noise_stddev = 0.5;

    auto msg = make_magnetic_field(to_ros_time(4.5), "robot_1/mag_link", mag);

    REQUIRE(msg.header.stamp == to_ros_time(4.5));
    REQUIRE(msg.header.frame_id == "robot_1/mag_link");
    // sim (Y-up, Z-forward) -> REP-103 (X-forward, Z-up): (x, y, z) -> (z, x, y).
    // microteslas -> tesla: * 1e-6.
    REQUIRE(msg.magnetic_field.x == Approx(21.130913e-6));
    REQUIRE(msg.magnetic_field.y == Approx(0.0).margin(1e-12));
    REQUIRE(msg.magnetic_field.z == Approx(-45.315389e-6));

    const double var = 0.5 * 0.5 * 1e-12;
    REQUIRE(msg.magnetic_field_covariance[0] == Approx(var));
    REQUIRE(msg.magnetic_field_covariance[4] == Approx(var));
    REQUIRE(msg.magnetic_field_covariance[8] == Approx(var));
    REQUIRE(msg.magnetic_field_covariance[1] == Approx(0.0));
}

TEST_CASE("make_camera_image copies RGB buffer with metadata", "[ros2]") {
    CameraSensor cam;
    cam.width = 160;
    cam.height = 120;
    cam.rebuild();
    cam.image_data[0] = 7;
    cam.image_data[1] = 8;

    auto msg = make_camera_image(to_ros_time(5.0), "robot_1/camera_link", cam);

    REQUIRE(msg.header.frame_id == "robot_1/camera_link");
    REQUIRE(msg.width == 160);
    REQUIRE(msg.height == 120);
    REQUIRE(msg.encoding == "rgb8");
    REQUIRE(msg.step == 160 * 3);
    REQUIRE(msg.data.size() == 160 * 120 * 3);
    REQUIRE(msg.data[0] == 7);
    REQUIRE(msg.data[1] == 8);
}

TEST_CASE("make_imu fills covariances from sensor noise", "[ros2]") {
    ImuSensor imu;
    imu.angular_velocity_noise_stddev = 0.1;
    imu.linear_acceleration_noise_stddev = 0.2;

    auto msg = make_imu(to_ros_time(3.0), "robot_1/imu_link", imu);

    REQUIRE(msg.header.frame_id == "robot_1/imu_link");
    // Gyro/accel variance comes from the sensor noise, so robot_localization
    // does not treat the (noisy) measurements as infinitely trustworthy.
    REQUIRE(msg.orientation_covariance[0] == Approx(1e-6));
    REQUIRE(msg.orientation_covariance[4] == Approx(1e-6));
    REQUIRE(msg.orientation_covariance[8] == Approx(1e-6));
    REQUIRE(msg.angular_velocity_covariance[0] == Approx(0.1 * 0.1));
    REQUIRE(msg.angular_velocity_covariance[4] == Approx(0.1 * 0.1));
    REQUIRE(msg.angular_velocity_covariance[8] == Approx(0.1 * 0.1));
    REQUIRE(msg.linear_acceleration_covariance[0] == Approx(0.2 * 0.2));
    REQUIRE(msg.linear_acceleration_covariance[4] == Approx(0.2 * 0.2));
    REQUIRE(msg.linear_acceleration_covariance[8] == Approx(0.2 * 0.2));
}

TEST_CASE("make_camera_info derives intrinsics from FOV", "[ros2]") {
    CameraSensor cam;
    cam.width = 200;
    cam.height = 100;
    cam.fov_deg = 90.0;

    auto msg = make_camera_info(to_ros_time(6.0), "robot_1/camera_link", cam);

    REQUIRE(msg.width == 200);
    REQUIRE(msg.height == 100);
    // The renderer applies fov_deg as the VERTICAL field of view, so
    // fx = fy = (h/2) / tan(vfov/2) = 50 for a 100 px height at 90 deg.
    REQUIRE(msg.k[0] == Approx(50.0));
    REQUIRE(msg.k[4] == Approx(50.0));
    REQUIRE(msg.k[2] == Approx(100.0));
    REQUIRE(msg.k[5] == Approx(50.0));
    REQUIRE(msg.distortion_model == "plumb_bob");
    REQUIRE(msg.d.size() == 5);
    REQUIRE(msg.r.size() == 9);
    REQUIRE(msg.r[0] == Approx(1.0));
    REQUIRE(msg.r[4] == Approx(1.0));
    REQUIRE(msg.r[8] == Approx(1.0));
}

TEST_CASE("make_depth_image copies float depth buffer", "[ros2]") {
    DepthCameraSensor depth;
    depth.width = 4;
    depth.height = 4;
    depth.rebuild();
    depth.depth_data[0] = 1.25f;
    depth.depth_data[1] = 0.5f;

    auto msg = make_depth_image(to_ros_time(7.0), "robot_1/depth_link", depth);

    REQUIRE(msg.header.frame_id == "robot_1/depth_link");
    REQUIRE(msg.encoding == "32FC1");
    REQUIRE(msg.step == 4 * 4);
    REQUIRE(msg.data.size() == 4 * 4 * sizeof(float));

    const float* pixels = reinterpret_cast<const float*>(msg.data.data());
    REQUIRE(pixels[0] == Approx(1.25));
    REQUIRE(pixels[1] == Approx(0.5));
}

TEST_CASE("make_camera_info derives depth intrinsics from FOV", "[ros2]") {
    DepthCameraSensor depth;
    depth.width = 160;
    depth.height = 120;
    depth.fov_deg = 90.0;

    auto msg = make_camera_info(to_ros_time(6.5), "robot_1/depth_link", depth);

    REQUIRE(msg.header.frame_id == "robot_1/depth_link");
    REQUIRE(msg.width == 160);
    REQUIRE(msg.height == 120);
    // fov_deg is the VERTICAL fov: fx = fy = (h/2) / tan(45 deg) = 60.
    REQUIRE(msg.k[0] == Approx(60.0));
    REQUIRE(msg.k[4] == Approx(60.0));
    REQUIRE(msg.k[2] == Approx(80.0));
    REQUIRE(msg.k[5] == Approx(60.0));
    REQUIRE(msg.distortion_model == "plumb_bob");
    REQUIRE(msg.d.size() == 5);
    REQUIRE(msg.r.size() == 9);
}

TEST_CASE("make_point_cloud2 emits NaN for no-detection rays", "[ros2]") {
    LidarSensor3D lidar;
    lidar.vertical_beams = 1;
    lidar.horizontal_rays = 2;
    lidar.rebuild();
    lidar.last_ranges[0] = 2.0f;
    lidar.last_ranges[1] = std::numeric_limits<float>::infinity();

    auto msg = make_point_cloud2(to_ros_time(8.0), "robot_1/lidar3d_link", lidar);

    REQUIRE(msg.header.frame_id == "robot_1/lidar3d_link");
    REQUIRE(msg.width == 2);
    REQUIRE(msg.point_step == 12);
    REQUIRE(msg.data.size() == 2 * 12);

    const float* p0 = reinterpret_cast<const float*>(msg.data.data());
    REQUIRE(std::isfinite(p0[0]));

    const float* p1 = reinterpret_cast<const float*>(msg.data.data() + 12);
    REQUIRE(std::isnan(p1[0]));
    REQUIRE(std::isnan(p1[1]));
    REQUIRE(std::isnan(p1[2]));
}

TEST_CASE("make_camera_optical_tf builds the optical frame transform", "[ros2]") {
    auto ts = make_camera_optical_tf("robot_1/camera_link", "robot_1/camera_optical_frame");

    REQUIRE(ts.header.stamp.sec == 0);
    REQUIRE(ts.header.frame_id == "robot_1/camera_link");
    REQUIRE(ts.child_frame_id == "robot_1/camera_optical_frame");
    REQUIRE(ts.transform.translation.x == Approx(0.0));
    REQUIRE(ts.transform.translation.y == Approx(0.0));
    REQUIRE(ts.transform.translation.z == Approx(0.0));

    // TF semantics: p_mount = R * p_optical, i.e. the rotation stored in the
    // transform expresses the optical frame axes in the mount frame. A camera
    // mount that is REP-103 X-forward/Y-left/Z-up must be rotated so that the
    // optical axes are: +Z out of the lens (= mount +X), +X image-right
    // (= mount -Y), +Y image-down (= mount -Z).
    Quaternion q(ts.transform.rotation.w, ts.transform.rotation.x, ts.transform.rotation.y,
                 ts.transform.rotation.z);

    Vec3 mount_z = q.rotate(Vec3(0.0, 0.0, 1.0));
    REQUIRE(mount_z.x == Approx(1.0));
    REQUIRE(mount_z.y == Approx(0.0).margin(1e-9));
    REQUIRE(mount_z.z == Approx(0.0).margin(1e-9));

    Vec3 mount_x = q.rotate(Vec3(1.0, 0.0, 0.0));
    REQUIRE(mount_x.x == Approx(0.0).margin(1e-9));
    REQUIRE(mount_x.y == Approx(-1.0));
    REQUIRE(mount_x.z == Approx(0.0).margin(1e-9));

    Vec3 mount_y = q.rotate(Vec3(0.0, 1.0, 0.0));
    REQUIRE(mount_y.x == Approx(0.0).margin(1e-9));
    REQUIRE(mount_y.y == Approx(0.0).margin(1e-9));
    REQUIRE(mount_y.z == Approx(-1.0));
}

TEST_CASE("ROS2Bridge publishes camera optical frames on /tf_static", "[ros2]") {
    World world;
    Entity robot = world.create_entity();
    world.add_component<Name>(robot, Name{"test_robot"});
    world.add_component<DifferentialDrive>(robot, DifferentialDrive{});
    world.add_component<CameraSensor>(robot, CameraSensor{});
    world.add_component<DepthCameraSensor>(robot, DepthCameraSensor{});
    world.add_component<MagnetometerSensor>(robot, MagnetometerSensor{});
    world.add_component<Transform3D>(robot, Transform3D{});

    ROS2Bridge bridge;
    auto listener = std::make_shared<rclcpp::Node>("tf_static_listener");
    tf2_msgs::msg::TFMessage received;
    auto sub = listener->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).transient_local(),
        [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) { received = *msg; });

    bridge.init(world);
    for (int i = 0; i < 100 && received.transforms.empty(); ++i) {
        rclcpp::spin_some(listener);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    bool found_cam_optical = false;
    bool found_depth_optical = false;
    for (const auto& t : received.transforms) {
        if (t.child_frame_id == "test_robot_1/camera_optical_frame") {
            REQUIRE(t.header.frame_id == "test_robot_1/camera_link");
            found_cam_optical = true;
        }
        if (t.child_frame_id == "test_robot_1/depth_optical_frame") {
            REQUIRE(t.header.frame_id == "test_robot_1/depth_link");
            found_depth_optical = true;
        }
    }

    bool found_mag = false;
    for (const auto& t : received.transforms) {
        if (t.child_frame_id == "test_robot_1/mag_link") {
            REQUIRE(t.header.frame_id == "test_robot_1/base_link");
            found_mag = true;
        }
    }
    REQUIRE(found_mag);

    REQUIRE(found_cam_optical);
    REQUIRE(found_depth_optical);
}

TEST_CASE("ROS2Bridge can omit the odom to base_footprint TF", "[ros2]") {
    World world;
    Entity robot = world.create_entity();
    world.add_component<Name>(robot, Name{"test_robot"});
    world.add_component<DifferentialDrive>(robot, DifferentialDrive{});
    world.add_component<Transform3D>(robot, Transform3D{});
    world.get_component<Transform3D>(robot)->position = Vec3(1.0, 0.5, 2.0);

    ROS2Bridge bridge;
    bridge.set_publish_odom_tf(false);
    bridge.init(world);
    bridge.set_sim_time(1.0, 0.01);
    bridge.start_publishing();

    auto listener = std::make_shared<rclcpp::Node>("tf_listener");
    tf2_msgs::msg::TFMessage received;
    auto sub = listener->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(10),
        [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) { received = *msg; });

    bool found_footprint_link = false;
    bool found_odom_footprint = false;
    for (int i = 0; i < 300 && !found_footprint_link; ++i) {
        rclcpp::spin_some(listener);
        for (const auto& t : received.transforms) {
            if (t.header.frame_id == "test_robot_1/base_footprint" &&
                t.child_frame_id == "test_robot_1/base_link") {
                found_footprint_link = true;
            }
            if (t.header.frame_id == "test_robot_1/odom" &&
                t.child_frame_id == "test_robot_1/base_footprint") {
                found_odom_footprint = true;
            }
        }
        received.transforms.clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // The EKF owns odom -> base_footprint, so the sim must not publish it.
    REQUIRE(found_footprint_link);
    REQUIRE(!found_odom_footprint);
}
