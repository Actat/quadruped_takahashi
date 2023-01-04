#include "quadruped_takahashi.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadruped_takahashi_node>());
  rclcpp::shutdown();
  return 0;
}

quadruped_takahashi_node::quadruped_takahashi_node()
    : Node("quadruped_takahashi_node") {
  tf_buffer_    = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  service_mode_ = this->create_service<quadruped_takahashi::srv::Mode>(
      "~/mode", std::bind(&quadruped_takahashi_node::callback_mode_, this,
                          std::placeholders::_1, std::placeholders::_2));

  client_b3m_mf_ = this->create_client<kondo_b3m_ros2::srv::MotorFree>(
      "/kondo_b3m_free_motor");
  client_b3m_spc_ =
      this->create_client<kondo_b3m_ros2::srv::StartPositionControl>(
          "/kondo_b3m_start_position_control");
  client_b3m_dp_ = this->create_client<kondo_b3m_ros2::srv::DesiredPosition>(
      "/kondo_b3m_desired_position");

  timer_            = nullptr;
  subscription_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&quadruped_takahashi_node::callback_imu_, this,
                std::placeholders::_1));
  publisher_tf_ =
      this->create_publisher<tf2_msgs::msg::TFMessage>("~/tf", rclcpp::QoS(10));
}

void quadruped_takahashi_node::callback_imu_(
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

  auto vec_lf4       = tfq_odom_base * vect_(tf_lf4);
  auto vec_rf4       = tfq_odom_base * vect_(tf_rf4);
  auto vec_lh4       = tfq_odom_base * vect_(tf_lh4);
  auto vec_rh4       = tfq_odom_base * vect_(tf_rh4);
  auto vec_odom_base = Eigen::Vector3d(
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

void quadruped_takahashi_node::callback_mode_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Request> const request,
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
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
void quadruped_takahashi_node::mode_motor_free_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
  auto req      = std::make_shared<kondo_b3m_ros2::srv::MotorFree::Request>();
  auto v        = std::vector<uint8_t>({255});
  req->data_len = 1;
  req->id       = v;
  if (!client_b3m_mf_->wait_for_service(
          std::chrono::duration<int, std::milli>(5))) {
    response->success = false;
    return;
  }
  response->success  = true;
  auto future_result = client_b3m_mf_->async_send_request(req);
}
void quadruped_takahashi_node::mode_start_position_control_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
  auto req =
      std::make_shared<kondo_b3m_ros2::srv::StartPositionControl::Request>();
  auto v        = std::vector<uint8_t>({255});
  req->data_len = 1;
  req->id       = v;
  if (!client_b3m_spc_->wait_for_service(
          std::chrono::duration<int, std::milli>(5))) {
    response->success = false;
    return;
  }
  response->success  = true;
  auto future_result = client_b3m_spc_->async_send_request(req);
}
void quadruped_takahashi_node::mode_stand_(
    std::shared_ptr<quadruped_takahashi::srv::Mode::Response> response) {
  timer_ = this->create_wall_timer(
      std::chrono::microseconds(10000),
      std::bind(&quadruped_takahashi_node::timer_callback_stand_, this));
  response->success = true;
  return;
}

void quadruped_takahashi_node::timer_callback_stand_() {
  auto ar_lf =
      ik_lf_(r_base_lf0 + Eigen::Vector3d(0, 0, -stand_hight + foot_radius));
  auto ar_rf =
      ik_rf_(r_base_rf0 + Eigen::Vector3d(0, 0, -stand_hight + foot_radius));
  auto ar_lh =
      ik_lh_(r_base_lh0 + Eigen::Vector3d(0, 0, -stand_hight + foot_radius));
  auto ar_rh =
      ik_rh_(r_base_rh0 + Eigen::Vector3d(0, 0, -stand_hight + foot_radius));

  auto arr = std::array<double, 12>({ar_lf.at(0), ar_lf.at(1), ar_lf.at(2),  //
                                     ar_rf.at(0), ar_rf.at(1), ar_rf.at(2),  //
                                     ar_lh.at(0), ar_lh.at(1), ar_lh.at(2),  //
                                     ar_rh.at(0), ar_rh.at(1), ar_rh.at(2)});
  auto req = std::make_shared<kondo_b3m_ros2::srv::DesiredPosition::Request>();
  auto v   = std::vector<kondo_b3m_ros2::msg::DesiredPosition>(12);
  for (int i = 0; i < 12; ++i) {
    v.at(i).id       = i;
    v.at(i).position = arr.at(i);
  }
  req->data_len = 12;
  req->position = v;
  auto res =
      [this](rclcpp::Client<kondo_b3m_ros2::srv::DesiredPosition>::SharedFuture
                 res) {
        auto response = res.get();
        if (!response->success) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to call service client 'client_b3m_dp_'.");
        }
      };
  if (!client_b3m_dp_->wait_for_service(
          std::chrono::duration<int, std::milli>(5))) {
    RCLCPP_WARN(this->get_logger(),
                "Service client 'client_b3m_dp_' is not available.");
    return;
  }
  auto future_result = client_b3m_dp_->async_send_request(req, res);
  return;
}

std::array<double, 3> quadruped_takahashi_node::ik_lf_(
    Eigen::Vector3d const &r_base_lf4) {
  auto r_lf0_lf4 = r_base_lf4 - r_base_lf0;
  auto ar        = ik_xf_(r_lf0_lf4);
  ar.at(0)       = clamp_(ar.at(0), -0.174532925, M_PI_4);
  ar.at(1)       = clamp_(ar.at(1), -M_PI_2, 0);
  ar.at(2)       = clamp_(ar.at(2), 0, 2.801777048);
  return ar;
}
std::array<double, 3> quadruped_takahashi_node::ik_rf_(
    Eigen::Vector3d const &r_base_rf4) {
  auto r_rf0_rf4 = r_base_rf4 - r_base_rf0;
  auto ar        = ik_xf_(r_rf0_rf4);
  ar.at(0)       = clamp_(ar.at(0), -M_PI_4, 0.174532925);
  ar.at(1)       = clamp_(ar.at(1), -M_PI_2, 0);
  ar.at(2)       = clamp_(ar.at(2), 0, 2.801777048);
  return ar;
}
std::array<double, 3> quadruped_takahashi_node::ik_lh_(
    Eigen::Vector3d const &r_base_lh4) {
  auto r_lh0_lh4 = r_base_lh4 - r_base_lh0;
  auto ar        = ik_xh_(r_lh0_lh4);
  ar.at(0)       = clamp_(ar.at(0), -0.174532925, M_PI_4);
  ar.at(1)       = clamp_(ar.at(1), 0, M_PI_2);
  ar.at(2)       = clamp_(ar.at(2), -2.801777048, 0);
  return ar;
}
std::array<double, 3> quadruped_takahashi_node::ik_rh_(
    Eigen::Vector3d const &r_base_rh4) {
  auto r_rh0_rh4 = r_base_rh4 - r_base_rh0;
  auto ar        = ik_xh_(r_rh0_rh4);
  ar.at(0)       = clamp_(ar.at(0), -M_PI_4, 0.174532925);
  ar.at(1)       = clamp_(ar.at(1), 0, M_PI_2);
  ar.at(2)       = clamp_(ar.at(2), -2.801777048, 0);
  return ar;
}
std::array<double, 3> quadruped_takahashi_node::ik_xf_(
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
std::array<double, 3> quadruped_takahashi_node::ik_xh_(
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
quadruped_takahashi_node::lookup_transform_(
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

Eigen::Quaterniond quadruped_takahashi_node::quat_(
    geometry_msgs::msg::TransformStamped const &tf) {
  return Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                            tf.transform.rotation.y, tf.transform.rotation.z);
}

Eigen::Quaterniond quadruped_takahashi_node::quat_(
    sensor_msgs::msg::Imu::SharedPtr const msg) {
  return Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                            msg->orientation.y, msg->orientation.z);
}

Eigen::Vector3d quadruped_takahashi_node::vect_(
    geometry_msgs::msg::TransformStamped const &tf) {
  return Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y,
                         tf.transform.translation.z);
}

double quadruped_takahashi_node::clamp_(double value, double low, double high) {
  return std::min(std::max(value, low), high);
}
