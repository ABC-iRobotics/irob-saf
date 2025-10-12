// src/irob_dvrk/robot_server_psm.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <irob_msgs/action/robot.hpp>
#include "irob_dvrk/arm_types.hpp"

using Robot = irob_msgs::action::Robot;

class RobotServerPSM : public rclcpp::Node
{
public:
  using GoalHandleRobot = rclcpp_action::ServerGoalHandle<Robot>;

  RobotServerPSM()
  : Node("robot_server_psm")
  {
    arm_name_ = this->declare_parameter<std::string>("arm_name", "psm1");

    auto qos = rclcpp::QoS(10).reliable();

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        arm_name_ + "/joint_states", qos);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        arm_name_ + "/tool_pose", qos);

    robot_server_ = rclcpp_action::create_server<Robot>(
      this,
      arm_name_ + "/robot_action",
      std::bind(&RobotServerPSM::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotServerPSM::handle_cancel, this, std::placeholders::_1),
      std::bind(&RobotServerPSM::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "RobotServerPSM for %s started", arm_name_.c_str());
  }

private:
  std::string arm_name_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp_action::Server<Robot>::SharedPtr robot_server_;

  // ------------------------------------------------------------------------
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const Robot::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "[%s] Received goal: action=%d, move_allowed=%s, #toolposes=%zu, dt=%.3f",
      arm_name_.c_str(),
      goal->action,
      goal->move_allowed ? "true" : "false",
      goal->trajectory.toolposes.size(),
      goal->trajectory.dt);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // ------------------------------------------------------------------------
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRobot> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "[%s] Cancel request", arm_name_.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // ------------------------------------------------------------------------
  void handle_accepted(const std::shared_ptr<GoalHandleRobot> goal_handle)
  {
    std::thread([this, goal_handle]()
    {
      auto result = std::make_shared<Robot::Result>();
      auto feedback = std::make_shared<Robot::Feedback>();

      RCLCPP_INFO(get_logger(), "[%s] Executing action...", arm_name_.c_str());
      rclcpp::Rate rate(50);

      for (int i = 0; i <= 100 && rclcpp::ok(); ++i)
      {
        // Publish dummy joint and pose data
        sensor_msgs::msg::JointState js;
        js.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
        js.position = {0.001*i, 0.001*i, 0.001*i, 0.0, 0.0, 0.0};
        js.header.stamp = now();
        joint_state_pub_->publish(js);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = now();
        pose_msg.pose.position.x = 0.001 * i;
        pose_msg.pose.orientation.w = 1.0;
        pose_pub_->publish(pose_msg);

        // Fill feedback (ToolPose)
        feedback->pose.transform.translation.x = pose_msg.pose.position.x;
        feedback->pose.transform.translation.y = pose_msg.pose.position.y;
        feedback->pose.transform.translation.z = pose_msg.pose.position.z;
        feedback->pose.transform.rotation = pose_msg.pose.orientation;
        feedback->pose.jaw = 0.0;
        feedback->info = "Moving...";
        goal_handle->publish_feedback(feedback);

        rate.sleep();
      }

      result->pose = feedback->pose;
      result->info = "Completed trajectory";

      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "[%s] Action done.", arm_name_.c_str());
    }).detach();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotServerPSM>());
  rclcpp::shutdown();
  return 0;
}
