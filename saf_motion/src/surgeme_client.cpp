#include <saf_motion/surgeme_client.hpp>

using namespace std::chrono_literals;

namespace saf {

SurgemeClient::SurgemeClient(const rclcpp::Node::SharedPtr &node,
                             const std::string &arm_name)
    : node_(node), arm_name_(arm_name) {
  ac_ = rclcpp_action::create_client<Surgeme>(node_, action_name_());
  create_publishers_();

  RCLCPP_INFO(node_->get_logger(),
              "[%s] SurgemeClient created, using action '%s'",
              arm_name_.c_str(), action_name_().c_str());
}

bool SurgemeClient::wait_for_server(std::chrono::milliseconds timeout) {
  RCLCPP_INFO(node_->get_logger(), "[%s] Waiting for Surgeme action server...",
              arm_name_.c_str());
  return ac_->wait_for_action_server(timeout);
}

void SurgemeClient::create_publishers_() {
  pose_cur_pub_ =
      node_->create_publisher<saf_msgs::msg::ToolPoseStamped>(
          "maneuver/" + arm_name_ + "/position_cartesian_current_cf", 10);

  js_cur_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "maneuver/" + arm_name_ + "/joint_state_current", 10);
}

void SurgemeClient::grasp(double effort) {
  Surgeme::Goal goal;

  goal.action = saf_msgs::action::Surgeme_Goal::GRASP;
  goal.speed_jaw = effort;
  goal.compression_rate = 1.0;

  if (!wait_for_server()) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Surgeme server not available, cannot send GRASP.",
                arm_name_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] Sending GRASP Surgeme", arm_name_.c_str());
  auto opts = rclcpp_action::Client<Surgeme>::SendGoalOptions{};
  ac_->async_send_goal(goal, opts);
}

void SurgemeClient::release() {
  Surgeme::Goal goal;
  goal.action = saf_msgs::action::Surgeme_Goal::RELEASE;
  goal.speed_jaw = 0.5;

  if (!wait_for_server()) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Surgeme server not available, cannot send RELEASE.",
                arm_name_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] Sending RELEASE Surgeme", arm_name_.c_str());
  ac_->async_send_goal(goal);
}

void SurgemeClient::nav_to_pos(const Eigen::Affine3d &tgt, double speed_ratio) {
  Surgeme::Goal goal;
  goal.action = saf_msgs::action::Surgeme_Goal::NAV_TO_POS;

  Eigen::Quaterniond q(tgt.rotation());
  Eigen::Vector3d t = tgt.translation();
  goal.target.translation.x = t.x();
  goal.target.translation.y = t.y();
  goal.target.translation.z = t.z();
  goal.target.rotation.x = q.x();
  goal.target.rotation.y = q.y();
  goal.target.rotation.z = q.z();
  goal.target.rotation.w = q.w();
  goal.speed_cartesian = speed_ratio;

  if (!wait_for_server()) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Surgeme server not available, cannot send NAV_TO_POS.",
                arm_name_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[%s] Sending NAV_TO_POS Surgeme (speed_ratio=%.2f)",
              arm_name_.c_str(), speed_ratio);
  ac_->async_send_goal(goal);
}

}  // namespace saf
