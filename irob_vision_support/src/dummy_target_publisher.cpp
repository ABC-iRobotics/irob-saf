#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class DummyTargetPublisher : public rclcpp::Node {
public:
    DummyTargetPublisher()
    : rclcpp::Node("dummy_target_publisher")
    {
        this->declare_parameter<std::string>("frame_id", "world");
        this->declare_parameter<double>("rate_hz", 10.0);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("rate_hz", rate_);

        pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/vision/target", 10);
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
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
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
    std::string child_frame_;
    double rate_{30.0};
    int marker_id_{0};

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyTargetPublisher>());
    rclcpp::shutdown();
    return 0;
}
