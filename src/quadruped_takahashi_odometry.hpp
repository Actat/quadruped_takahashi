#ifndef QUADRUPED_TAKAHASHI_ODOMETRY_HPP_
#define QUADRUPED_TAKAHASHI_ODOMETRY_HPP_

#include <array>
#include <chrono>
#include <eigen3/Eigen/Geometry>
#include "geometry_msgs/msg/point_stamped.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class quadruped_takahashi_odometry : public rclcpp::Node {
public:
  double const length_t            = 0.1;
  double const length_s            = 0.1;
  double const foot_radius         = 0.01;
  double const stand_hight         = 0.15;
  Eigen::Vector3d const r_base_lf0 = Eigen::Vector3d(0.076, 0.04, 0);
  Eigen::Vector3d const r_base_rf0 = Eigen::Vector3d(0.076, -0.04, 0);
  Eigen::Vector3d const r_base_lh0 = Eigen::Vector3d(-0.076, 0.04, 0);
  Eigen::Vector3d const r_base_rh0 = Eigen::Vector3d(-0.076, -0.04, 0);

  quadruped_takahashi_odometry();

private:
  std::chrono::duration<int, std::milli> const control_period_ =
      std::chrono::duration<int, std::milli>(10);
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void callback_imu_(sensor_msgs::msg::Imu::SharedPtr const msg);

  double clamp_(double value, double low, double high);
};

#endif  // QUADRUPED_TAKAHASHI_ODOMETRY_HPP_
