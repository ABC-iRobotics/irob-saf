// robot_client.cpp (ROS 2)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <saf_msgs/action/robot.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class RobotClient {
public:
  using Robot = saf_msgs::action::Robot;

  RobotClient(const rclcpp::Node::SharedPtr& node,
              const std::string& arm_name,
              double dt)
  : node_(node), arm_name_(arm_name), dt_(dt)
  {
    auto qos = rclcpp::QoS(10).reliable();

    joint_state_current_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("robot/joint_states_current", qos);

    joint_state_current_sub_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
        "robot/joint_states",
        qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg){
          joint_state_current_ = *msg;
          joint_state_current_pub_->publish(*msg);
        });

    action_client_ = rclcpp_action::create_client<Robot>(
      node_, "robot/" + arm_name_ + "/robot_action");
  }

  bool send_goal(const Robot::Goal& goal)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "Robot action server not available");
      return false;
    }

    auto options = rclcpp_action::Client<Robot>::SendGoalOptions();
    options.feedback_callback =
      [this](auto, const std::shared_ptr<const Robot::Feedback> feedback){
        // use feedback
        (void)feedback;
      };

    auto future_handle = action_client_->async_send_goal(goal, options);
    auto goal_handle = future_handle.get();
    if (!goal_handle) return false;

    auto future_result = action_client_->async_get_result(goal_handle);
    auto result = future_result.get();
    return result.code == rclcpp_action::ResultCode::SUCCEEDED;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string arm_name_;
  double dt_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_current_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_current_sub_;
  sensor_msgs::msg::JointState joint_state_current_;
  rclcpp_action::Client<Robot>::SharedPtr action_client_;
};
