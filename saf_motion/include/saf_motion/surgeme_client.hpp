#pragma once
/*
 *  surgeme_client.hpp (ROS 2)
 *  Single-arm action client to saf_msgs::Surgeme
 */

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <saf_msgs/action/surgeme.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <saf_msgs/msg/tool_pose_stamped.hpp>
#include <saf_msgs/msg/instrument_info.hpp>

namespace saf {

/**
 * @brief ROS 2 action client wrapper to send high-level surgical maneuvers (Surgemes)
 *        to the iROB framework. It publishes live tool and joint states and exposes
 *        convenient helper methods for grasp, release and navigation commands.
 */
class SurgemeClient {
public:
  using Surgeme = saf_msgs::action::Surgeme;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Surgeme>;

  explicit SurgemeClient(const rclcpp::Node::SharedPtr &node,
                         const std::string &arm_name);

  // --------------------------------------------------------------------------
  // High-level public API (mirrors ROS1 behavior)
  // --------------------------------------------------------------------------
  void grasp(double effort = 1.0);
  void release();
  void nav_to_pos(const Eigen::Affine3d &tgt, double speed_ratio = 1.0);

  // --------------------------------------------------------------------------
  // Utilities
  // --------------------------------------------------------------------------
  bool wait_for_server(std::chrono::milliseconds timeout =
                           std::chrono::milliseconds(2000));

private:
  // Internal members
  rclcpp::Node::SharedPtr node_;
  std::string arm_name_;
  rclcpp_action::Client<Surgeme>::SharedPtr ac_;

  // Publishers for diagnostics
  rclcpp::Publisher<saf_msgs::msg::ToolPoseStamped>::SharedPtr pose_cur_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_cur_pub_;

  void create_publishers_();
  std::string action_name_() const { return "surgeme/" + arm_name_; }
};

}  // namespace saf
