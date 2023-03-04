#include "quadruped_takahashi_control.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadruped_takahashi_control_node>());
  rclcpp::shutdown();
  return 0;
}

quadruped_takahashi_control_node::quadruped_takahashi_control_node()
    : Node("quadruped_takahashi_control_node") {
  tf_buffer_    = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  service_mode_ = this->create_service<quadruped_takahashi::srv::Mode>(
      "~/mode", std::bind(&quadruped_takahashi_control_node::callback_mode_,
                          this, std::placeholders::_1, std::placeholders::_2));

  client_b3m_mode_ = this->create_client<kondo_b3m_ros2::srv::ControlMode>(
      "/kondo_b3m/control_mode");
  client_b3m_desired_ =
      this->create_client<kondo_b3m_ros2::srv::Desired>("/kondo_b3m/desired");
  timer_ = nullptr;
}

void quadruped_takahashi_control_node::callback_mode_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Request> const request,
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Mode change to " + request->data);
  timer_ = nullptr;
  if (request->data == "motor_free") {
    mode_motor_free_(response);
    return;
  } else if (request->data == "start_position_control") {
    mode_start_position_control_(response);
    return;
  } else if (request->data == "stand") {
    mode_stand_(response);
    return;
  }

  response->success = false;
  response->message = "Unknown command";
  return;
}
void quadruped_takahashi_control_node::mode_motor_free_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
  auto req  = std::make_shared<kondo_b3m_ros2::srv::ControlMode::Request>();
  req->name = joint_list_;
  req->mode = std::vector<std::string>(
      {"f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f", "f"});
  if (!client_b3m_mode_->wait_for_service(
          std::chrono::duration<int, std::milli>(5))) {
    RCLCPP_WARN(this->get_logger(), "Failed to free b3m motor.");
    response->success = false;
    return;
  }
  response->success  = true;
  auto future_result = client_b3m_mode_->async_send_request(req);
}
void quadruped_takahashi_control_node::mode_start_position_control_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
  auto req  = std::make_shared<kondo_b3m_ros2::srv::ControlMode::Request>();
  req->name = joint_list_;
  req->mode = std::vector<std::string>(
      {"p", "p", "p", "p", "p", "p", "p", "p", "p", "p", "p", "p"});
  if (!client_b3m_mode_->wait_for_service(
          std::chrono::duration<int, std::milli>(5))) {
    response->success = false;
    return;
  }
  response->success  = true;
  auto future_result = client_b3m_mode_->async_send_request(req);
}
void quadruped_takahashi_control_node::mode_stand_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
  timer_ = this->create_wall_timer(
      control_period_,
      std::bind(&quadruped_takahashi_control_node::timer_callback_stand_,
                this));
  response->success = true;
  return;
}

void quadruped_takahashi_control_node::timer_callback_stand_() {
  auto now     = this->get_clock()->now();
  auto timeout = tf2::durationFromSec(1.0);
  geometry_msgs::msg::TransformStamped tf_base, tf_lf0, tf_rf0, tf_lh0, tf_rh0;
  try {
    tf_base = lookup_transform_("map", "base_link", now, timeout);
    tf_lf0  = lookup_transform_("base_link", "lfleg0", now, timeout);
    tf_rf0  = lookup_transform_("base_link", "rfleg0", now, timeout);
    tf_lh0  = lookup_transform_("base_link", "lhleg0", now, timeout);
    tf_rh0  = lookup_transform_("base_link", "rhleg0", now, timeout);
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

  auto diff_lf0 = Eigen::Vector3d(
      0, 0, (vect_(tf_lf0) - quat_(tf_base) * vect_(tf_lf0)).z());
  auto diff_rf0 = Eigen::Vector3d(
      0, 0, (vect_(tf_rf0) - quat_(tf_base) * vect_(tf_rf0)).z());
  auto diff_lh0 = Eigen::Vector3d(
      0, 0, (vect_(tf_lh0) - quat_(tf_base) * vect_(tf_lh0)).z());
  auto diff_rh0 = Eigen::Vector3d(
      0, 0, (vect_(tf_rh0) - quat_(tf_base) * vect_(tf_rh0)).z());

  double const leg_opening_width = 0.02;
  auto static feedback_lf        = Eigen::Vector3d(0, 0, 0);
  auto static feedback_rf        = Eigen::Vector3d(0, 0, 0);
  auto static feedback_lh        = Eigen::Vector3d(0, 0, 0);
  auto static feedback_rh        = Eigen::Vector3d(0, 0, 0);
  auto static diff_lf0_last      = Eigen::Vector3d(0, 0, 0);
  auto static diff_lf0_last      = Eigen::Vector3d(0, 0, 0);
  auto static diff_lf0_last      = Eigen::Vector3d(0, 0, 0);
  auto static diff_lf0_last      = Eigen::Vector3d(0, 0, 0);
  double const Kp                = 1.0;
  double const Ki                = 0.0;

  feedback_lf = feedback_lf +
                (Kp + 0.001 * control_period_.count() * Ki) * diff_lf0 -
                Kp * diff_lf0_last;
  feedback_rf = feedback_rf +
                (Kp + 0.001 * control_period_.count() * Ki) * diff_rf0 -
                Kp * diff_lf0_last;
  feedback_lh = feedback_lh +
                (Kp + 0.001 * control_period_.count() * Ki) * diff_lh0 -
                Kp * diff_lf0_last;
  feedback_rh = feedback_rh +
                (Kp + 0.001 * control_period_.count() * Ki) * diff_rh0 -
                Kp * diff_lf0_last;
  diff_lf0_last = diff_lf0;
  diff_lf0_last = diff_rf0;
  diff_lf0_last = diff_lh0;
  diff_lf0_last = diff_rh0;

  auto ar_lf = ik_lf_(  //
      r_base_lf0 +
      Eigen::Vector3d(0, leg_opening_width, -stand_hight + foot_radius) +
      feedback_lf);
  auto ar_rf = ik_rf_(  //
      r_base_rf0 +
      Eigen::Vector3d(0, -leg_opening_width, -stand_hight + foot_radius) +
      feedback_rf);
  auto ar_lh = ik_lh_(  //
      r_base_lh0 +
      Eigen::Vector3d(0, leg_opening_width, -stand_hight + foot_radius) +
      feedback_lh);
  auto ar_rh = ik_rh_(  //
      r_base_rh0 +
      Eigen::Vector3d(0, -leg_opening_width, -stand_hight + foot_radius) +
      feedback_rh);

  auto req   = std::make_shared<kondo_b3m_ros2::srv::Desired::Request>();
  req->name  = joint_list_;
  req->value = std::vector<double>({ar_lf.at(0), ar_lf.at(1), ar_lf.at(2),  //
                                    ar_rf.at(0), ar_rf.at(1), ar_rf.at(2),  //
                                    ar_lh.at(0), ar_lh.at(1), ar_lh.at(2),  //
                                    ar_rh.at(0), ar_rh.at(1), ar_rh.at(2)});
  auto res =
      [this](rclcpp::Client<kondo_b3m_ros2::srv::Desired>::SharedFuture res) {
        auto response = res.get();
        if (!response->success) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to call service client 'client_b3m_dp_'.");
        }
      };
  if (!client_b3m_desired_->wait_for_service(
          std::chrono::duration<int, std::milli>(5))) {
    RCLCPP_WARN(this->get_logger(),
                "Service client 'client_b3m_dp_' is not available.");
    return;
  }
  auto future_result = client_b3m_desired_->async_send_request(req, res);
  return;
}

std::array<double, 3> quadruped_takahashi_control_node::ik_lf_(
    Eigen::Vector3d const &r_base_lf4) {
  auto r_lf0_lf4 = r_base_lf4 - r_base_lf0;
  auto ar        = ik_xf_(r_lf0_lf4);
  ar.at(0)       = clamp_(ar.at(0), -0.174532925, M_PI_4);
  ar.at(1)       = clamp_(ar.at(1), -M_PI_2, 0);
  ar.at(2)       = clamp_(ar.at(2), 0, 2.801777048);
  return ar;
}
std::array<double, 3> quadruped_takahashi_control_node::ik_rf_(
    Eigen::Vector3d const &r_base_rf4) {
  auto r_rf0_rf4 = r_base_rf4 - r_base_rf0;
  auto ar        = ik_xf_(r_rf0_rf4);
  ar.at(0)       = clamp_(ar.at(0), -M_PI_4, 0.174532925);
  ar.at(1)       = clamp_(ar.at(1), -M_PI_2, 0);
  ar.at(2)       = clamp_(ar.at(2), 0, 2.801777048);
  return ar;
}
std::array<double, 3> quadruped_takahashi_control_node::ik_lh_(
    Eigen::Vector3d const &r_base_lh4) {
  auto r_lh0_lh4 = r_base_lh4 - r_base_lh0;
  auto ar        = ik_xh_(r_lh0_lh4);
  ar.at(0)       = clamp_(ar.at(0), -0.174532925, M_PI_4);
  ar.at(1)       = clamp_(ar.at(1), 0, M_PI_2);
  ar.at(2)       = clamp_(ar.at(2), -2.801777048, 0);
  return ar;
}
std::array<double, 3> quadruped_takahashi_control_node::ik_rh_(
    Eigen::Vector3d const &r_base_rh4) {
  auto r_rh0_rh4 = r_base_rh4 - r_base_rh0;
  auto ar        = ik_xh_(r_rh0_rh4);
  ar.at(0)       = clamp_(ar.at(0), -M_PI_4, 0.174532925);
  ar.at(1)       = clamp_(ar.at(1), 0, M_PI_2);
  ar.at(2)       = clamp_(ar.at(2), -2.801777048, 0);
  return ar;
}
std::array<double, 3> quadruped_takahashi_control_node::ik_xf_(
    Eigen::Vector3d const &r_xf0_xf4) {
  auto r         = r_xf0_xf4.norm() <= length_t + length_s
                       ? r_xf0_xf4
                       : (length_t + length_s) * r_xf0_xf4.normalized();
  double const x = r.x();
  double const y = r.y();
  double const z = r.z();

  double theta_xx0 = y == 0 && z == 0 ? 0 : std::atan2(y, -z);
  double theta_xx2 = std::acos(
      (x * x + y * y + z * z - length_t * length_t - length_s * length_s) /
      (2.0 * length_t * length_s));
  double theta_xx1 = std::atan2(
      (x * (-length_t - length_s * std::cos(theta_xx2)) +
       std::sqrt(y * y + z * z) * (-length_s * std::sin(theta_xx2))) /
          (length_t * length_t + 2 * length_t * length_s * std::cos(theta_xx2) +
           length_s * length_s),
      (x * (-length_s * std::sin(theta_xx2)) +
       std::sqrt(y * y + z * z) * (length_t + length_s * std::cos(theta_xx2))) /
          (length_t * length_t + 2 * length_t * length_s * std::cos(theta_xx2) +
           length_s * length_s));

  std::array<double, 3> ar = {theta_xx0, theta_xx1, theta_xx2};
  return ar;
}
std::array<double, 3> quadruped_takahashi_control_node::ik_xh_(
    Eigen::Vector3d const &r_xh0_xh4) {
  auto r         = r_xh0_xh4.norm() <= length_t + length_s
                       ? r_xh0_xh4
                       : (length_t + length_s) * r_xh0_xh4.normalized();
  double const x = r.x();
  double const y = r.y();
  double const z = r.z();

  double theta_xx0 = y == 0 && z == 0 ? 0 : std::atan2(y, -z);
  double theta_xx2 = -std::acos(
      (x * x + y * y + z * z - length_t * length_t - length_s * length_s) /
      (2.0 * length_t * length_s));
  double theta_xx1 = std::atan2(
      (x * (-length_t - length_s * std::cos(theta_xx2)) +
       std::sqrt(y * y + z * z) * (-length_s * std::sin(theta_xx2))) /
          (length_t * length_t + 2 * length_t * length_s * std::cos(theta_xx2) +
           length_s * length_s),
      (x * (-length_s * std::sin(theta_xx2)) +
       std::sqrt(y * y + z * z) * (length_t + length_s * std::cos(theta_xx2))) /
          (length_t * length_t + 2 * length_t * length_s * std::cos(theta_xx2) +
           length_s * length_s));

  std::array<double, 3> ar = {theta_xx0, theta_xx1, theta_xx2};
  return ar;
}

geometry_msgs::msg::TransformStamped
quadruped_takahashi_control_node::lookup_transform_(
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

Eigen::Quaterniond quadruped_takahashi_control_node::quat_(
    geometry_msgs::msg::TransformStamped const &tf) {
  return Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                            tf.transform.rotation.y, tf.transform.rotation.z);
}

Eigen::Vector3d quadruped_takahashi_control_node::vect_(
    geometry_msgs::msg::TransformStamped const &tf) {
  return Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y,
                         tf.transform.translation.z);
}

double quadruped_takahashi_control_node::clamp_(double value,
                                                double low,
                                                double high) {
  return std::min(std::max(value, low), high);
}
