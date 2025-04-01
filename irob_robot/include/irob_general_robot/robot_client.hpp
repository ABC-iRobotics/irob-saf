/*
 * 	robot_client.hpp
 *
 *	Author(s): Tamas Levendovics
 *	Created on: 2017-07-19
 *
 *	Single arm, ROS actions interface to the robot server. This objects are
 *  planned to be used as members of irob_motion/SurgemeServer.
 *	Substribes to cartesian_pos.
 *	Move relative, absolute, gripper, waypoints, Bezier...
 */

#ifndef ROBOT_CLIENT_HPP_
#define ROBOT_CLIENT_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <irob_utils/tool_pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/trajectory_factory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/irob_action_client.hpp>

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

#include <irob_msgs/msg/tool_pose_stamped.hpp>
#include <irob_msgs/action/robot_action.hpp>
#include <irob_msgs/msg/instrument_info.hpp>
#include <irob_msgs/msg/instrument_jaw_part.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace saf {

class RobotClient {

public:
  enum class CoordFrame { WCS, TCPF };

private:
  const std::string arm_name;
  rclcpp::Node::SharedPtr node;

  double dt;

  // Action clients
  IrobActionClient<irob_msgs::action::RobotAction> ac;

  // States
  irob_msgs::msg::ToolPoseStamped position_cartesian_current;
  sensor_msgs::msg::JointState joint_state_current;
  irob_msgs::msg::InstrumentInfo instrument_info;

  // Subscribers
  rclcpp::Subscription<irob_msgs::msg::ToolPoseStamped>::SharedPtr position_cartesian_current_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_current_sub;
  rclcpp::Subscription<irob_msgs::msg::InstrumentInfo>::SharedPtr instrument_info_sub;

  // Publishers
  rclcpp::Publisher<irob_msgs::msg::ToolPoseStamped>::SharedPtr position_cartesian_current_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_current_pub;
  rclcpp::Publisher<irob_msgs::msg::InstrumentInfo>::SharedPtr instrument_info_pub;

  void subscribeTopics();
  void advertiseTopics();
  void waitForActionServer();

public:
  RobotClient(rclcpp::Node::SharedPtr, std::string, double);
  ~RobotClient();

  // Callbacks
  void positionCartesianCurrentCB(const irob_msgs::msg::ToolPoseStamped::SharedPtr msg);
  void jointStateCurrentCB(const sensor_msgs::msg::JointState::SharedPtr msg);
  void instrumentInfoCB(const irob_msgs::msg::InstrumentInfo::SharedPtr msg);

  ToolPose getPoseCurrent();
  sensor_msgs::msg::JointState getJointStateCurrent();
  irob_msgs::msg::InstrumentInfo getInstrumentInfo();
  std::string getName();

  // Robot motions
  void resetPose(bool);
  void moveJaws(double, double);
  void moveTool(ToolPose, double, std::vector<ToolPose> = std::vector<ToolPose>(), InterpolationMethod = LINEAR);
  void moveJoints(sensor_msgs::msg::JointState);

  void stop();

  bool isActionDone(bool = true);
  action_msgs::msg::GoalStatus getState();

  irob_msgs::msg::RobotFeedback getFeedback(bool = true);
  irob_msgs::msg::RobotResult getResult(bool = true);
};

}  // namespace saf

#endif /* ROBOT_CLIENT_HPP_ */
