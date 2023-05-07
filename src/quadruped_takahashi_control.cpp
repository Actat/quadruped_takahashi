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
  auto now = this->get_clock()->now();
  geometry_msgs::msg::Vector3Stamped v_lf, v_rf, v_lh, v_rh;
  v_lf.header.stamp = v_rf.header.stamp =  //
      v_lh.header.stamp = v_rh.header.stamp = now;
  v_lf.header.frame_id                      = "lfleg0";
  v_rf.header.frame_id                      = "rfleg0";
  v_lh.header.frame_id                      = "lhleg0";
  v_rh.header.frame_id                      = "rhleg0";
  v_lf.vector.x                             = r_base_lf0(0);
  v_lf.vector.y                             = r_base_lf0(1);
  v_lf.vector.z                             = r_base_lf0(2);
  v_rf.vector.x                             = r_base_rf0(0);
  v_rf.vector.y                             = r_base_rf0(1);
  v_rf.vector.z                             = r_base_rf0(2);
  v_lh.vector.x                             = r_base_lh0(0);
  v_lh.vector.y                             = r_base_lh0(1);
  v_lh.vector.z                             = r_base_lh0(2);
  v_rh.vector.x                             = r_base_rh0(0);
  v_rh.vector.y                             = r_base_rh0(1);
  v_rh.vector.z                             = r_base_rh0(2);
  auto timeout                              = tf2::durationFromSec(1.0);
  try {
    v_lf = tf_buffer_->transform(v_lf, "map", timeout);
    v_rf = tf_buffer_->transform(v_rf, "map", timeout);
    v_lh = tf_buffer_->transform(v_lh, "map", timeout);
    v_rh = tf_buffer_->transform(v_rh, "map", timeout);
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

  auto X_lf0 = Eigen::Vector3d(0, 0, -v_lf.vector.z);
  auto X_rf0 = Eigen::Vector3d(0, 0, -v_rf.vector.z);
  auto X_lh0 = Eigen::Vector3d(0, 0, -v_lh.vector.z);
  auto X_rh0 = Eigen::Vector3d(0, 0, -v_rh.vector.z);

  auto static X_lf0_last = Eigen::Vector3d(0, 0, 0);
  auto static X_rf0_last = Eigen::Vector3d(0, 0, 0);
  auto static X_lh0_last = Eigen::Vector3d(0, 0, 0);
  auto static X_rh0_last = Eigen::Vector3d(0, 0, 0);
  auto static Yi_lf      = Eigen::Vector3d(0, 0, 0);
  auto static Yi_rf      = Eigen::Vector3d(0, 0, 0);
  auto static Yi_lh      = Eigen::Vector3d(0, 0, 0);
  auto static Yi_rh      = Eigen::Vector3d(0, 0, 0);
  auto static Yd_lf      = Eigen::Vector3d(0, 0, 0);
  auto static Yd_rf      = Eigen::Vector3d(0, 0, 0);
  auto static Yd_lh      = Eigen::Vector3d(0, 0, 0);
  auto static Yd_rh      = Eigen::Vector3d(0, 0, 0);

  double const Kp  = 0.85;
  double const Ti  = 0.2;
  double const Td  = 0.000000;
  double const T   = control_period_.count() / 1000.0;
  double const eta = 0.1;

  auto Yp_lf = Kp * X_lf0;
  auto Yp_rf = Kp * X_rf0;
  auto Yp_lh = Kp * X_lh0;
  auto Yp_rh = Kp * X_rh0;
  Yi_lf      = Yi_lf + Kp * T / Ti * X_lf0;
  Yi_rf      = Yi_rf + Kp * T / Ti * X_rf0;
  Yi_lh      = Yi_lh + Kp * T / Ti * X_lh0;
  Yi_rh      = Yi_rh + Kp * T / Ti * X_rh0;
  Yd_lf      = (eta * Td) / (T + eta * Td) * Yd_lf +
          (Kp * Td) / (T + eta * Td) * X_lf0 -
          (Kp * Td) / (T + eta * Td) * X_lf0_last;
  Yd_rf = (eta * Td) / (T + eta * Td) * Yd_rf +
          (Kp * Td) / (T + eta * Td) * X_rf0 -
          (Kp * Td) / (T + eta * Td) * X_rf0_last;
  Yd_lh = (eta * Td) / (T + eta * Td) * Yd_lh +
          (Kp * Td) / (T + eta * Td) * X_lh0 -
          (Kp * Td) / (T + eta * Td) * X_lh0_last;
  Yd_rh = (eta * Td) / (T + eta * Td) * Yd_rh +
          (Kp * Td) / (T + eta * Td) * X_rh0 -
          (Kp * Td) / (T + eta * Td) * X_rh0_last;
  auto Y_lf = Yp_lf + Yi_lf + Yd_lf;
  auto Y_rf = Yp_rf + Yi_rf + Yd_rf;
  auto Y_lh = Yp_lh + Yi_lh + Yd_lh;
  auto Y_rh = Yp_rh + Yi_rh + Yd_rh;

  X_lf0_last = X_lf0;
  X_rf0_last = X_rf0;
  X_lh0_last = X_lh0;
  X_rh0_last = X_rh0;

  double const leg_opening_width = 0.02;
  auto ar_lf =
      ik_lf_(-Y_lf + r_base_lf0 +
             Eigen::Vector3d(0, leg_opening_width, -stand_hight + foot_radius));
  auto ar_rf = ik_rf_(
      -Y_rf + r_base_rf0 +
      Eigen::Vector3d(0, -leg_opening_width, -stand_hight + foot_radius));
  auto ar_lh =
      ik_lh_(-Y_lh + r_base_lh0 +
             Eigen::Vector3d(0, leg_opening_width, -stand_hight + foot_radius));
  auto ar_rh = ik_rh_(
      -Y_rh + r_base_rh0 +
      Eigen::Vector3d(0, -leg_opening_width, -stand_hight + foot_radius));

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

double quadruped_takahashi_control_node::clamp_(double value,
                                                double low,
                                                double high) {
  return std::min(std::max(value, low), high);
}
