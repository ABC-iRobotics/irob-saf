#pragma once
/*
 *  surgeme_server.hpp (ROS 2)
 *  Action server implementing surgemes (grasp, release, nav_to_pos, etc.)
 */

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <saf_msgs/action/surgeme.hpp>
#include <saf_msgs/msg/instrument_info.hpp>

namespace saf {

    class SurgemeServer : public rclcpp::Node {
    public:
        using Surgeme = saf_msgs::action::Surgeme;
        using GoalHandle = rclcpp_action::ServerGoalHandle<Surgeme>;

        explicit SurgemeServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        rclcpp_action::Server<Surgeme>::SharedPtr server_;
        std::string arm_name_;
        double loop_rate_hz_;
        bool simulated_;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                                std::shared_ptr<const Surgeme::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

        // Surgeme implementations (stubs)
        void do_grasp(const std::shared_ptr<GoalHandle>& gh);
        void do_release(const std::shared_ptr<GoalHandle>& gh);
        void do_nav_to_pos(const std::shared_ptr<GoalHandle>& gh);

        // Helpers
        void declare_and_get_params_();
    };

} // namespace saf
