/*
 *  dummy_target_publisher.cpp
 *
 *  Author(s): Tamas Levendovics
 *  Created on: 2016-10-26
 *  ROS 2 port: 2025-10-14
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

namespace saf
{
    class DummyTargetPublisher : public rclcpp::Node {
    public:
        DummyTargetPublisher()
        : rclcpp::Node("dummy_target_publisher")
        {
            this->declare_parameter<std::string>("frame_id", "world");
            this->declare_parameter<double>("rate_hz", 10.0);
            this->get_parameter("frame_id", frame_id_);
            this->get_parameter("rate_hz", rate_);

            // Define the rotation axis and angle, for "natutal" PSM orientation
            Eigen::Vector3d axis = Eigen::Vector3d(-1.0 / sqrt(2.0), -1.0 / sqrt(2.0), 0.0).normalized();
            double angle = M_PI;  // 180 degrees
            marker_orientation_ = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));

            pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
            timer_ = this->create_wall_timer(
              std::chrono::duration<double>(1.0 / rate_),
              std::bind(&DummyTargetPublisher::tick_, this));
            RCLCPP_INFO(get_logger(), "dummy_target_publisher running @ %.1f Hz", rate_);
        }

    private:
        void tick_() {
            auto now = this->get_clock()->now();
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = now;
            marker.header.frame_id = frame_id_;
            marker.ns = "dvrk_viz";
            marker.id = marker_id_;
            marker_id_++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = -0.05;
            marker.pose.position.y = 0.08;
            marker.pose.position.z = -0.12;
            marker.pose.orientation.x = marker_orientation_.x();
            marker.pose.orientation.y = marker_orientation_.y();
            marker.pose.orientation.z = marker_orientation_.z();
            marker.pose.orientation.w = marker_orientation_.w();
            marker.scale.x = 0.005;
            marker.scale.y = 0.005;
            marker.scale.z = 0.005;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            pub_->publish(marker);
        }

        std::string frame_id_;
        double rate_{30.0};
        int marker_id_{0};
        Eigen::Quaterniond marker_orientation_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

    };
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<saf::DummyTargetPublisher>());
    rclcpp::shutdown();
    return 0;
}

