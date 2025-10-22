/*
 * 	robot_server.hpp
 *
 *	Author(s): Tamas Levendovics
 *	Created on: 2016-11-07
 *
 *  Abstract base class for robot servers, interfacing saf_msgs/Robot actions to the actual robots.
 *
 *  Important note: initRosCommunication must be called
 *  in child class or main function!
 *
 */
#ifndef ROBOT_SERVER_HPP_
#define ROBOT_SERVER_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <saf_utils/tool_pose.hpp>
#include <saf_utils/trajectory.hpp>
#include <saf_utils/utils.hpp>
#include <saf_utils/topic_name_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <saf_msgs/msg/tool_pose_stamped.hpp>
#include <saf_msgs/action/robot_action.hpp>
#include <saf_msgs/msg/instrument_info.hpp>
#include <saf_msgs/msg/instrument_jaw_part.hpp>

namespace saf {

class RobotServer {

public:
  // Constants
  static const bool ACTIVE = true;
  static const bool PASSIVE = false;
  static constexpr const double INFO_PUB_RATE = 10.0;

protected:
  const std::string arm_name;
  rclcpp::Node::SharedPtr node;
  bool isActive;

  // Action server
  std::shared_ptr<rclcpp_action::Server<saf_msgs::action::RobotAction>> action_server;

  // Hand-eye registration
  Eigen::Vector3d t;
  Eigen::Matrix3d R;
  Eigen::Affine3d T_he;

  // Surgical instrument information
  saf_msgs::msg::InstrumentInfo instrument_info;

  // Publishers
  rclcpp::Publisher<saf_msgs::msg::ToolPoseStamped>::SharedPtr measured_cp_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr measured_js_pub;
  rclcpp::Publisher<saf_msgs::msg::InstrumentInfo>::SharedPtr instrument_info_pub;

  virtual void subscribeLowLevelTopics() = 0;
  virtual void advertiseLowLevelTopics() = 0;

  void advertiseHighLevelTopics() {
    // robot interface
    measured_cp_pub = node->create_publisher<saf_msgs::msg::ToolPoseStamped>(
        "robot/" + arm_name + "/position_cartesian_current_cf", 10);
    measured_js_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "robot/" + arm_name + "/joint_state_current", 10);
    instrument_info_pub = node->create_publisher<saf_msgs::msg::InstrumentInfo>(
        "robot/" + arm_name + "/instrument_info", 10);
  }

  void startActionServer() {
    // Action server creation and spin
    auto handle_goal = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<saf_msgs::action::RobotAction>> goal_handle) {
      this->robotActionCB(goal_handle);
    };

    action_server = rclcpp_action::create_server<saf_msgs::action::RobotAction>(
        node, "robot/" + arm_name + "/robot_action",
        handle_goal, nullptr, nullptr);
  }

  void publishInfo() {
    rclcpp::Rate loop_rate(INFO_PUB_RATE);
    while (rclcpp::ok()) {
      instrument_info_pub->publish(instrument_info);
      loop_rate.sleep();
    }
  }

public:
  void initRosCommunication() {
    subscribeLowLevelTopics();
    if (isActive == ACTIVE) {
      advertiseLowLevelTopics();
    }

    advertiseHighLevelTopics();
    startActionServer();
    std::thread(&RobotServer::publishInfo, this).detach();
  }

  void loadRegistration(rclcpp::Node::SharedPtr priv_nh) {
    std::vector<double> param_t;
    priv_nh->get_parameter("t", param_t);

    std::vector<double> param_R;
    priv_nh->get_parameter("R", param_R);

    for (int i = 0; i < t.rows(); i++) {
      t(i) = param_t[i];
    }

    for (int i = 0; i < R.rows(); i++) {
      for (int j = 0; j < R.cols(); j++) {
        R(i, j) = param_R[(i * R.rows()) + j];
      }
    }

    T_he = Eigen::Translation3d(t) * Eigen::Affine3d(R) * Eigen::Scaling(0.001);

    RCLCPP_INFO_STREAM(
        priv_nh->get_logger(),
        "Registration read: " << std::endl << t << std::endl << R);
  }

  void loadInstrumentInfo(rclcpp::Node::SharedPtr priv_nh) {
    priv_nh->get_parameter("instrument/name", instrument_info.name);
    priv_nh->get_parameter("instrument/jaw_length", instrument_info.jaw_length);

    std::string basic_type;
    priv_nh->get_parameter("instrument/basic_type", basic_type);

    if (basic_type == "GRIPPER") {
      instrument_info.basic_type = saf_msgs::msg::InstrumentInfo::GRIPPER;
    } else if (basic_type == "SCISSORS") {
      instrument_info.basic_type = saf_msgs::msg::InstrumentInfo::SCISSORS;
    } else if (basic_type == "CAMERA") {
      instrument_info.basic_type = saf_msgs::msg::InstrumentInfo::CAMERA;
    } else {
      throw std::runtime_error("Invalid basic_type read from instrument info file.");
    }

    int i = 0;
    double probe;
    while (priv_nh->get_parameter("instrument/jaw_parts/p" + std::to_string(i) + "/start", probe)) {
      saf_msgs::msg::InstrumentJawPart jaw_part;
      priv_nh->get_parameter("instrument/jaw_parts/p" + std::to_string(i) + "/start", jaw_part.start);
      priv_nh->get_parameter("instrument/jaw_parts/p" + std::to_string(i) + "/end", jaw_part.end);
      std::string type;
      priv_nh->get_parameter("instrument/jaw_parts/p" + std::to_string(i) + "/type", type);

      if (type == "JOINT") {
        jaw_part.type = saf_msgs::msg::InstrumentJawPart::JOINT;
      } else if (type == "GRIPPER") {
        jaw_part.type = saf_msgs::msg::InstrumentJawPart::GRIPPER;
      } else if (type == "SCISSORS") {
        jaw_part.type = saf_msgs::msg::InstrumentJawPart::SCISSORS;
      } else {
        throw std::runtime_error("Invalid instrument part type read from instrument info file.");
      }

      instrument_info.jaw_parts.push_back(jaw_part);
      i++;
    }

    RCLCPP_INFO_STREAM(
        priv_nh->get_logger(),
        "Instrument info read: " << std::endl << instrument_info);
  }

  virtual void resetPose(bool) = 0;
  virtual void stop() = 0;
  virtual void followTrajectory(Trajectory<ToolPose>) = 0;
  virtual void moveJointAbsolute(sensor_msgs::msg::JointState, double) = 0;
  virtual ToolPose getPoseCurrent() = 0;

  virtual void robotActionCB(const std::shared_ptr<rclcpp_action::ServerGoalHandle<saf_msgs::action::RobotAction>> goal_handle) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Start robotActionCB");
    auto goal = goal_handle->get_goal();
    switch (goal->action) {
      case saf_msgs::action::RobotAction::Goal::STOP:
        stop();
        break;
      case saf_msgs::action::RobotAction::Goal::RESET_POSE:
        resetPose(goal->move_allowed);
        break;
      case saf_msgs::action::RobotAction::Goal::FOLLOW_TRAJECTORY:
        followTrajectory(goal->trajectory);
        break;
      case saf_msgs::action::RobotAction::Goal::MOVE_JOINT:
        moveJointAbsolute(goal->joint_state, 0.01);
        break;
      default:
        RCLCPP_ERROR_STREAM(node->get_logger(),
            arm_name << ": invalid robot action code received");
        saf_msgs::action::RobotAction::Result result;
        result.pose = getPoseCurrent().toRosToolPose();
        result.info = "invalid robot action code";
        goal_handle->abort(result);
        break;
    }
  }

  // initRosCommunication must be called in child class or main function
  RobotServer(rclcpp::Node::SharedPtr nh, rclcpp::Node::SharedPtr priv_nh,
              std::string arm_name, bool isActive)
    : node(nh), arm_name(arm_name), isActive(isActive) {
    loadRegistration(priv_nh);
    loadInstrumentInfo(priv_nh);
  }

  virtual ~RobotServer() {}
};

}  // namespace saf

#endif /* ROBOT_SERVER_HPP_ */
