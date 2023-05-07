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
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void quadruped_takahashi_odometry::callback_imu_(
    sensor_msgs::msg::Imu::SharedPtr const msg) {
  auto tfq_odom_base =
      Eigen::Quaterniond(0, 0, 1, 0) *
      Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                         msg->orientation.y, msg->orientation.z) *
      Eigen::Quaterniond(std::cos(-M_PI_4), 0, 0, std::sin(-M_PI_4));

  geometry_msgs::msg::PointStamped ps_lf, ps_rf, ps_lh, ps_rh;
  ps_lf.header.stamp = ps_rf.header.stamp =  //
      ps_lh.header.stamp = ps_rh.header.stamp = msg->header.stamp;
  ps_lf.header.frame_id                       = "lfleg4";
  ps_rf.header.frame_id                       = "rfleg4";
  ps_lh.header.frame_id                       = "lhleg4";
  ps_rh.header.frame_id                       = "rhleg4";
  ps_lf.point.x = ps_rf.point.x = ps_lh.point.x = ps_rh.point.x = 0.;
  ps_lf.point.y = ps_rf.point.y = ps_lh.point.y = ps_rh.point.y = 0.;
  ps_lf.point.z = ps_rf.point.z = ps_lh.point.z = ps_rh.point.z = 0.;
  auto timeout = tf2::durationFromSec(1.0);
  try {
    ps_lf = tf_buffer_->transform(ps_lf, "base_link", timeout);
    ps_rf = tf_buffer_->transform(ps_rf, "base_link", timeout);
    ps_lh = tf_buffer_->transform(ps_lh, "base_link", timeout);
    ps_rh = tf_buffer_->transform(ps_rh, "base_link", timeout);
  } catch (tf2::LookupException const &e) {
    RCLCPP_WARN(this->get_logger(), e.what());
    return;
  } catch (tf2::ConnectivityException const &e) {
    RCLCPP_WARN(this->get_logger(), e.what());
    return;
  } catch (tf2::ExtrapolationException const &e) {
    RCLCPP_WARN(this->get_logger(), e.what());
    return;
  } catch (tf2::InvalidArgumentException const &e) {
    RCLCPP_WARN(this->get_logger(), e.what());
    return;
  }

  auto vec_lf4 = tfq_odom_base *
                 Eigen::Vector3d(ps_lf.point.x, ps_lf.point.y, ps_lf.point.z);
  auto vec_rf4 = tfq_odom_base *
                 Eigen::Vector3d(ps_rf.point.x, ps_rf.point.y, ps_rf.point.z);
  auto vec_lh4 = tfq_odom_base *
                 Eigen::Vector3d(ps_lh.point.x, ps_lh.point.y, ps_lh.point.z);
  auto vec_rh4 = tfq_odom_base *
                 Eigen::Vector3d(ps_rh.point.x, ps_rh.point.y, ps_rh.point.z);
  auto vec_odom_base = Eigen::Vector3d(
      0, 0,
      foot_radius -
          std::min({vec_lf4.z(), vec_rf4.z(), vec_lh4.z(), vec_rh4.z()}));

  geometry_msgs::msg::TransformStamped tfs;
  tfs.header                  = msg->header;
  tfs.header.frame_id         = "odom";
  tfs.child_frame_id          = "base_link";
  tfs.transform.translation.x = vec_odom_base.x();
  tfs.transform.translation.y = vec_odom_base.y();
  tfs.transform.translation.z = vec_odom_base.z();
  tfs.transform.rotation.w    = tfq_odom_base.w();
  tfs.transform.rotation.x    = tfq_odom_base.x();
  tfs.transform.rotation.y    = tfq_odom_base.y();
  tfs.transform.rotation.z    = tfq_odom_base.z();
  tf_broadcaster_->sendTransform(tfs);
}
