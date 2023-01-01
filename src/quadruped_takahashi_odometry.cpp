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
  auto tfq_odom_base =
      Eigen::Quaterniond(0, 0, 1, 0) * quat_(msg) *
      Eigen::Quaterniond(std::cos(-M_PI_4), 0, 0, std::sin(-M_PI_4));

  auto stamp   = msg->header.stamp;
  auto timeout = tf2::durationFromSec(1.0);
  auto tf_lf4  = lookup_transform_("base_link", "lfleg4", stamp, timeout);
  auto tf_rf4  = lookup_transform_("base_link", "rfleg4", stamp, timeout);
  auto tf_lh4  = lookup_transform_("base_link", "lhleg4", stamp, timeout);
  auto tf_rh4  = lookup_transform_("base_link", "rhleg4", stamp, timeout);

  double const foot_radius = 0.01;
  auto vec_lf4             = tfq_odom_base * vect_(tf_lf4);
  auto vec_rf4             = tfq_odom_base * vect_(tf_rf4);
  auto vec_lh4             = tfq_odom_base * vect_(tf_lh4);
  auto vec_rh4             = tfq_odom_base * vect_(tf_rh4);
  auto vec_odom_base       = Eigen::Vector3d(
      0, 0,
      foot_radius -
          std::min({vec_lf4.z(), vec_rf4.z(), vec_lh4.z(), vec_rh4.z()}));

  auto tfmsg                    = tf2_msgs::msg::TFMessage();
  tfmsg.transforms              = {geometry_msgs::msg::TransformStamped()};
  tfmsg.transforms.at(0).header = msg->header;
  tfmsg.transforms.at(0).header.frame_id = "odom";
  tfmsg.transforms.at(0).child_frame_id  = "base_link";
  tfmsg.transforms.at(0).transform       = geometry_msgs::msg::Transform();
  tfmsg.transforms.at(0).transform.translation = geometry_msgs::msg::Vector3();
  tfmsg.transforms.at(0).transform.translation.x = vec_odom_base.x();
  tfmsg.transforms.at(0).transform.translation.y = vec_odom_base.y();
  tfmsg.transforms.at(0).transform.translation.z = vec_odom_base.z();
  tfmsg.transforms.at(0).transform.rotation = geometry_msgs::msg::Quaternion();
  tfmsg.transforms.at(0).transform.rotation.w = tfq_odom_base.w();
  tfmsg.transforms.at(0).transform.rotation.x = tfq_odom_base.x();
  tfmsg.transforms.at(0).transform.rotation.y = tfq_odom_base.y();
  tfmsg.transforms.at(0).transform.rotation.z = tfq_odom_base.z();
  publisher_tf_->publish(tfmsg);
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

Eigen::Quaterniond quadruped_takahashi_odometry::quat_(
    geometry_msgs::msg::TransformStamped const &tf) {
  return Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                            tf.transform.rotation.y, tf.transform.rotation.z);
}

Eigen::Quaterniond quadruped_takahashi_odometry::quat_(
    sensor_msgs::msg::Imu::SharedPtr const msg) {
  return Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                            msg->orientation.y, msg->orientation.z);
}

Eigen::Vector3d quadruped_takahashi_odometry::vect_(
    geometry_msgs::msg::TransformStamped const &tf) {
  return Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y,
                         tf.transform.translation.z);
}
