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
    sensor_msgs::msg::Imu::SharedPtr const msg) {
  auto stamp   = msg->header.stamp;
  auto timeout = tf2::durationFromSec(1.0);
  auto tf_lf4  = lookup_transform_("lfleg4", "base_link", stamp, timeout);
  auto tf_rf4  = lookup_transform_("rfleg4", "base_link", stamp, timeout);
  auto tf_lh4  = lookup_transform_("lhleg4", "base_link", stamp, timeout);
  auto tf_rh4  = lookup_transform_("rhleg4", "base_link", stamp, timeout);
}

geometry_msgs::msg::TransformStamped
quadruped_takahashi_odometry::lookup_transform_(
    std::string const target_frame,
    std::string const source_frame,
    builtin_interfaces::msg::Time const &time_stamp,
    tf2::Duration const &timeout) {
  return tf_buffer_->lookupTransform(
      target_frame, source_frame,
      tf2::TimePoint(std::chrono::seconds(time_stamp.sec) +
                     std::chrono::nanoseconds(time_stamp.nanosec)),
      timeout);
}
