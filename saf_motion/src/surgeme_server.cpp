#include <saf_motion/surgeme_server.hpp>

using namespace std::chrono_literals;

namespace saf {

SurgemeServer::SurgemeServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("surgeme_server", options)
{
  declare_and_get_params_();

  server_ = rclcpp_action::create_server<Surgeme>(
      this,
      "surgeme/" + arm_name_,
      std::bind(&SurgemeServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SurgemeServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SurgemeServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Surgeme server for arm '%s' started (simulated=%s, loop=%.1f Hz)",
              arm_name_.c_str(), simulated_ ? "true" : "false", loop_rate_hz_);
}

void SurgemeServer::declare_and_get_params_() {
  this->declare_parameter<std::string>("arm_name", "psm1");
  this->declare_parameter<double>("loop_rate_hz", 10.0);
  this->declare_parameter<bool>("simulated", false);

  this->get_parameter("arm_name", arm_name_);
  this->get_parameter("loop_rate_hz", loop_rate_hz_);
  this->get_parameter("simulated", simulated_);
}

rclcpp_action::GoalResponse
SurgemeServer::handle_goal(const rclcpp_action::GoalUUID&,
                           std::shared_ptr<const Surgeme::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received Surgeme goal action=%d", goal->action);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
SurgemeServer::handle_cancel(const std::shared_ptr<GoalHandle>)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel Surgeme goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SurgemeServer::handle_accepted(const std::shared_ptr<GoalHandle> gh)
{
  // Run each goal in its own thread
  std::thread([this, gh]() {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<Surgeme::Result>();

    switch (goal->action) {
      case saf_msgs::action::Surgeme_Goal::GRASP:       do_grasp(gh); break;
      case saf_msgs::action::Surgeme_Goal::RELEASE:     do_release(gh); break;
      case saf_msgs::action::Surgeme_Goal::NAV_TO_POS:  do_nav_to_pos(gh); break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unsupported surgeme action id: %d", goal->action);
        break;
    }

    result->info = "Surgeme completed successfully.";
    gh->succeed(result);
  }).detach();
}

void SurgemeServer::do_grasp(const std::shared_ptr<GoalHandle>& gh) {
  RCLCPP_INFO(get_logger(), "[%s] Executing GRASP", arm_name_.c_str());
  (void)gh;
  std::this_thread::sleep_for(200ms);
}

void SurgemeServer::do_release(const std::shared_ptr<GoalHandle>& gh) {
  RCLCPP_INFO(get_logger(), "[%s] Executing RELEASE", arm_name_.c_str());
  (void)gh;
  std::this_thread::sleep_for(200ms);
}

void SurgemeServer::do_nav_to_pos(const std::shared_ptr<GoalHandle>& gh) {
  RCLCPP_INFO(get_logger(), "[%s] Executing NAV_TO_POS", arm_name_.c_str());
  (void)gh;
  std::this_thread::sleep_for(500ms);
}

} // namespace saf

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<saf::SurgemeServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
