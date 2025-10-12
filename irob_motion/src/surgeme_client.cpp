#include <irob_motion/surgeme_client.hpp>

using namespace std::chrono_literals;

namespace saf {

SurgemeClient::SurgemeClient(const rclcpp::Node::SharedPtr& node,
                             const std::string& arm_name)
  : node_(node), arm_name_(arm_name)
{
  ac_ = rclcpp_action::create_client<Surgeme>(node_, action_name_());
  create_publishers_();
}

bool SurgemeClient::wait_for_server(std::chrono::milliseconds timeout) {
  return ac_->wait_for_action_server(timeout);
}

void SurgemeClient::create_publishers_() {
  pose_cur_pub_ = node_->create_publisher<irob_msgs::msg::ToolPoseStamped>(
      "maneuver/" + arm_name_ + "/position_cartesian_current_cf", 10);
  js_cur_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "maneuver/" + arm_name_ + "/joint_state_current", 10);
}

void SurgemeClient::grasp(double effort) {
  auto goal = Surgeme::Goal();

  goal.action = irob_msgs::action::Surgeme_Goal::GRASP;
  goal.speed_jaw = effort;        // use jaw speed
  goal.compression_rate = 1.0;    // default compression

  if (!wait_for_server()) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Surgeme server not available, cannot send GRASP.",
                arm_name_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] Sending GRASP surgeme", arm_name_.c_str());
  auto send_goal_opts = rclcpp_action::Client<Surgeme>::SendGoalOptions{};
  ac_->async_send_goal(goal, send_goal_opts);
}

void SurgemeClient::release() {
  auto goal = Surgeme::Goal();
  goal.action = irob_msgs::action::Surgeme_Goal::RELEASE;
  goal.speed_jaw = 0.5;

  if (!wait_for_server()) {
    RCLCPP_WARN(node_->get_logger(),
                "[%s] Surgeme server not available, cannot send RELEASE.",
                arm_name_.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] Sending RELEASE surgeme", arm_name_.c_str());
  ac_->async_send_goal(goal);
}

void SurgemeClient::nav_to_pos(const Eigen::Affine3d& tgt, double speed_ratio) {
  auto goal = Surgeme::Goal();
  goal.action = irob_msgs::action::Surgeme_Goal::NAV_TO_POS;

  // Convert Eigen transform to geometry_msgs::Transform
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

  RCLCPP_INFO(node_->get_logger(), "[%s] Sending NAV_TO_POS surgeme", arm_name_.c_str());
  ac_->async_send_goal(goal);
}

}  // namespace saf
