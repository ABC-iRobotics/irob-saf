/*
 *  robot_server.hpp
 *
 *  Author(s): Tamas Levendovics
 *	Created on: 2016-11-07
 *  ROS 2 port: 2025-10-14
 *
 *  Abstract base class for robot servers, interfacing saf_msgs/Robot actions to the actual robots.
 *
 *  Important note: initRosCommunication must be called
 *  in child class or main function!
 *
 */

#pragma once

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
#include <yaml-cpp/yaml.h>
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

class RobotServer : public rclcpp::Node {

public:
  // Constants
  static const bool ACTIVE = true;
  static const bool PASSIVE = false;
  static constexpr const double INFO_PUB_RATE = 10.0;

protected:
  const std::string arm_name;
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
    measured_cp_pub = this->create_publisher<saf_msgs::msg::ToolPoseStamped>(
        "robot/" + arm_name + "/position_cartesian_current_cf", 10);
    measured_js_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        "robot/" + arm_name + "/joint_state_current", 10);
    instrument_info_pub = this->create_publisher<saf_msgs::msg::InstrumentInfo>(
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

  void loadRegistration() {
    auto param_t = get_parameter("t").as_double_array();
    auto param_R = get_parameter("R").as_double_array();

    for (int i = 0; i < t.rows(); i++) {
      t(i) = param_t[i];
    }

    for (int i = 0; i < R.rows(); i++) {
      for (int j = 0; j < R.cols(); j++) {
        R(i, j) = param_R[(i * R.rows()) + j];
      }
    }

    T_he = Eigen::Translation3d(t) * Eigen::Affine3d(R) * Eigen::Scaling(0.001);

    RCLCPP_INFO(get_logger(), "Registration read: " << std::endl << t << std::endl << R);
  }

  void loadInstrumentInfo() {
    auto param_instrument_yaml = get_parameter("instrument_yaml").as_string();

    YAML::Node config = YAML::LoadFile(param_instrument_yaml);
    auto instr = config["instrument"];
    instrument_info.name = instr["name"].as<std::string>();
    auto basic_type = instr["basic_type"].as<std::string>();
    if (basic_type == "GRIPPER") {
      instrument_info.basic_type = saf_msgs::msg::InstrumentInfo::GRIPPER;
    } else if (basic_type == "SCISSORS") {
      instrument_info.basic_type = saf_msgs::msg::InstrumentInfo::SCISSORS;
    } else if (basic_type == "CAMERA") {
      instrument_info.basic_type = saf_msgs::msg::InstrumentInfo::CAMERA;
    } else {
      throw std::runtime_error("Invalid basic_type read from instrument info file.");
    }

    instrument_info.jaw_length = instr["jaw_length"].as<double>();

    for (auto part : instr["jaw_parts"])
    {
      saf_msgs::msg::InstrumentJawPart jaw_part;
      auto id = part.first.as<std::string>();
      auto p = part.second;
      jaw_part.start = p["start"].as<double>();
      jaw_part.end = p["end"].as<double>();

      auto type = p["type"].as<std::string>();
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
    }

    RCLCPP_INFO_STREAM(get_logger(), "Instrument info read: " << std::endl << instrument_info);
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
  RobotServer(std::string arm_name, bool isActive)
    : rclcpp::Node("robot_server_" + arm_name), arm_name(arm_name), isActive(isActive) {
      this->declare_parameter("t",  std::vector<double>(3, 0.0));
      std::vector<double> default_R = {0.1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
      this->declare_parameter("R",  default_R);
      this->declare_parameter("instrument_yaml",  "default");

      loadRegistration();
      loadInstrumentInfo();
  }

  virtual ~RobotServer() {}
};

}  // namespace saf

