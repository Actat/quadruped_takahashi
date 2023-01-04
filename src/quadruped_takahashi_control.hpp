#ifndef QUADRUPED_TAKAHASHI_CONTROL_HPP_
#define QUADRUPED_TAKAHASHI_CONTROL_HPP_

#include <array>
#include <chrono>
#include <eigen3/Eigen/Geometry>
#include "kondo_b3m_ros2/srv/desired_position.hpp"
#include "kondo_b3m_ros2/srv/motor_free.hpp"
#include "kondo_b3m_ros2/srv/start_position_control.hpp"
#include "quadruped_takahashi/srv/mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class quadruped_takahashi_control_node : public rclcpp::Node {
public:
  double const length_t            = 0.1;
  double const length_s            = 0.1;
  double const foot_radius         = 0.01;
  double const stand_hight         = 0.15;
  Eigen::Vector3d const r_base_lf0 = Eigen::Vector3d(0.076, 0.04, 0);
  Eigen::Vector3d const r_base_rf0 = Eigen::Vector3d(0.076, -0.04, 0);
  Eigen::Vector3d const r_base_lh0 = Eigen::Vector3d(-0.076, 0.04, 0);
  Eigen::Vector3d const r_base_rh0 = Eigen::Vector3d(-0.076, -0.04, 0);

  quadruped_takahashi_control_node();

private:
  std::chrono::duration<int, std::milli> const control_period_ =
      std::chrono::duration<int, std::milli>(10);
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<quadruped_takahashi::srv::Mode>::SharedPtr service_mode_;

  rclcpp::Client<kondo_b3m_ros2::srv::MotorFree>::SharedPtr client_b3m_mf_;
  rclcpp::Client<kondo_b3m_ros2::srv::StartPositionControl>::SharedPtr
      client_b3m_spc_;
  rclcpp::Client<kondo_b3m_ros2::srv::DesiredPosition>::SharedPtr
      client_b3m_dp_;

  void callback_mode_(
      std::shared_ptr<quadruped_takahashi::srv::Mode::Request> const request,
      std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response);
  void mode_motor_free_(
      std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response);
  void mode_start_position_control_(
      std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response);
  void mode_stand_(
      std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response);
  void timer_callback_stand_();

  std::array<double, 3> ik_lf_(Eigen::Vector3d const &r_base_lf4);
  std::array<double, 3> ik_rf_(Eigen::Vector3d const &r_base_rf4);
  std::array<double, 3> ik_lh_(Eigen::Vector3d const &r_base_lh4);
  std::array<double, 3> ik_rh_(Eigen::Vector3d const &r_base_rh4);
  std::array<double, 3> ik_xf_(Eigen::Vector3d const &r_xf0_rf4);
  std::array<double, 3> ik_xh_(Eigen::Vector3d const &r_xh0_lh4);

  geometry_msgs::msg::TransformStamped lookup_transform_(
      std::string const target_frame,
      std::string const source_frame,
      builtin_interfaces::msg::Time const &time_stamp,
      tf2::Duration const &timeout);
  Eigen::Quaterniond quat_(geometry_msgs::msg::TransformStamped const &tf);
  Eigen::Vector3d vect_(geometry_msgs::msg::TransformStamped const &tf);

  double clamp_(double value, double low, double high);
};

#endif  // QUADRUPED_TAKAHASHI_CONTROL_HPP_
