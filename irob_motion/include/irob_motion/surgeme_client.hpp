#pragma once
/*
 *  surgeme_client.hpp (ROS 2)
 *  Single-arm action client to irob_msgs::Surgeme
 */

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <irob_msgs/action/surgeme.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <irob_msgs/msg/tool_pose_stamped.hpp>
#include <irob_msgs/msg/instrument_info.hpp>

namespace saf {

    class SurgemeClient {
    public:
        using Surgeme = irob_msgs::action::Surgeme;
        using GoalHandle = rclcpp_action::ClientGoalHandle<Surgeme>;

        explicit SurgemeClient(const rclcpp::Node::SharedPtr& node, const std::string& arm_name);

        // Example public API (mirror your ROS1 methods; signatures kept where possible)
        void grasp(double effort = 1.0);
        void release();
        void nav_to_pos(const Eigen::Affine3d& tgt, double speed_ratio = 1.0);

        // Convenience
        bool wait_for_server(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));

    private:
        rclcpp::Node::SharedPtr node_;
        std::string arm_name_;
        rclcpp_action::Client<Surgeme>::SharedPtr ac_;

        // publishers similar to ROS1 (adjust topics as needed)
        rclcpp::Publisher<irob_msgs::msg::ToolPoseStamped>::SharedPtr pose_cur_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_cur_pub_;

        void create_publishers_();
        std::string action_name_() const { return "surgeme/" + arm_name_; }
    };

} // namespace saf
