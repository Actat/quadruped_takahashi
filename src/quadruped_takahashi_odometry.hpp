#ifndef QUADRUPED_TAKAHASHI_ODOMETRY_HPP_
#define QUADRUPED_TAKAHASHI_ODOMETRY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class quadruped_takahashi_odometry : public rclcpp::Node {
public:
  quadruped_takahashi_odometry();

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_tf_;

  void callback_imu_(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif  // QUADRUPED_TAKAHASHI_ODOMETRY_HPP_
