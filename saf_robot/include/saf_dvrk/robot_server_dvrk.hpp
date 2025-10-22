/*
 * 	robot_server_dvrk.hpp
 *
 *	Author(s): Tamas Levendovics
 *	Created on: 2016-11-07
 *
 *  Base class for dVRK robot arms, itself usable for
 *  MTMs and ECM.
 *
 */

#ifndef ROBOT_SERVER_DVRK_HPP_
#define ROBOT_SERVER_DVRK_HPP_

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <saf_dvrk/arm_types.hpp>
#include <saf_general_robot/robot_server.hpp>

namespace saf {

class RobotServerDVRK: public RobotServer {

public:

  // Constants
  static const std::string READY;

protected:
  const ArmTypes arm_typ;

  // States
  std_msgs::msg::String status;
  sensor_msgs::msg::JointState measured_js;
  geometry_msgs::msg::TransformStamped measured_cp;
  std_msgs::msg::String error;
  std_msgs::msg::String warning;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr smeasured_js_sub;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr measured_cp_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr error_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr warning_sub;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr position_joint_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr position_cartesian_pub;

  void subscribeLowLevelTopics();
  void advertiseLowLevelTopics();

public:
  RobotServerDVRK(rclcpp::Node::SharedPtr node, ArmTypes arm_type, std::string name, bool use_sim);
  ~RobotServerDVRK();

  // Callbacks
  void resetPose(bool);
  void stop();
  void followTrajectory(Trajectory<ToolPose>);
  void moveJointAbsolute(sensor_msgs::msg::JointState joint_state, double velocity);

  void status_cb(const std_msgs::msg::String::SharedPtr msg);
  void error_cb(const std_msgs::msg::String::SharedPtr msg);
  void warning_cb(const std_msgs::msg::String::SharedPtr msg);
  void measured_js_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  virtual void measured_cp_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

  void loadRegistration(std::string);

  double getJointStateCurrent(int);
  std::vector<double> getJointStateCurrent();
  Eigen::Vector3d getPositionCartesianCurrent();
  Eigen::Quaternion<double> getOrientationCartesianCurrent();
  ToolPose getPoseCurrent();

  // DVRK actions
  std::string getCurrentState();
  void moveCartesianRelative(Eigen::Translation3d, double = 0.01);
  virtual void moveCartesianAbsolute(ToolPose, double = 0.01);

  void recordTrajectory(Trajectory<Eigen::Vector3d>&);
  void recordTrajectory(Trajectory<ToolPose>&);
  void saveTrajectory(std::string);

  void checkErrors();
  void checkVelCartesian(const ToolPose&, const ToolPose&, double);
  void checkNaNCartesian(const ToolPose&);
  void checkVelJoint(const sensor_msgs::msg::JointState&,
                     const std::vector<double>&, double);
  sensor_msgs::msg::JointState maximizeVelJoint(const sensor_msgs::msg::JointState&,
                     const std::vector<double>&, double);
  void checkNaNJoint(const sensor_msgs::msg::JointState&);

};

}  // namespace saf

#endif /* ROBOT_SERVER_DVRK_HPP_ */

