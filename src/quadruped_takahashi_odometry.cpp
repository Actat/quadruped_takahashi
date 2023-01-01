#include "quadruped_takahashi_odometry.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadruped_takahashi_odometry>());
  rclcpp::shutdown();
  return 0;
}

quadruped_takahashi_odometry::quadruped_takahashi_odometry()
    : Node("quadruped_takahashi_odometry") {
  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&quadruped_takahashi_odometry::callback_imu_, this,
                std::placeholders::_1));
  publisher_tf_ =
      this->create_publisher<tf2_msgs::msg::TFMessage>("~/tf", rclcpp::QoS(10));
}

void quadruped_takahashi_odometry::callback_imu_(
    const sensor_msgs::msg::Imu::SharedPtr msg) {}
