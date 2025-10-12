// robot_server_dvrk.cpp (ROS 2)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <irob_msgs/action/robot.hpp>  // generated from Robot.action

class RobotServerDVRK : public rclcpp::Node {
public:
  using Robot = irob_msgs::action::Robot;
  using GoalHandleRobot = rclcpp_action::ServerGoalHandle<Robot>;

  RobotServerDVRK()
  : Node("robot_server_dvrk")
  {
    // QoS: keep last few, reliable
    auto qos = rclcpp::QoS(10).reliable();

    // pubs/subs example
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "robot/joint_states", qos);

    // action server
    robot_action_server_ = rclcpp_action::create_server<Robot>(
      this,
      "robot/" + arm_name_ + "/robot_action",
      std::bind(&RobotServerDVRK::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotServerDVRK::handle_cancel, this, std::placeholders::_1),
      std::bind(&RobotServerDVRK::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp_action::Server<Robot>::SharedPtr robot_action_server_;
  std::string arm_name_{"psm1"};

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const Robot::Goal> goal)
  {
    (void)goal;
    RCLCPP_INFO(get_logger(), "Received Robot goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRobot> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRobot> goal_handle)
  {
    // Do work on a separate thread or a timer to avoid blocking executor
    std::thread([this, goal_handle]() {
      auto result = std::make_shared<Robot::Result>();
      auto feedback_msg = std::make_shared<Robot::Feedback>();
      rclcpp::Rate r(100);
      bool success = true;

      // ... perform motion, fill feedback_msg fields ...
      for (int i = 0; rclcpp::ok() && i < 50; ++i) {
        feedback_msg->progress = i / 50.0;
        goal_handle->publish_feedback(feedback_msg);
        r.sleep();
      }

      if (success) {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Robot action succeeded");
      } else {
        goal_handle->abort(result);
      }
    }).detach();
  }
};
