#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

class DummyTargetPublisher : public rclcpp::Node {
public:
    DummyTargetPublisher()
    : rclcpp::Node("dummy_target_publisher"),
      tf_broadcaster_(this)
    {
        this->declare_parameter<std::string>("parent_frame", "camera_frame");
        this->declare_parameter<std::string>("child_frame", "target_frame");
        this->declare_parameter<double>("rate_hz", 30.0);
        this->get_parameter("parent_frame", parent_frame_);
        this->get_parameter("child_frame", child_frame_);
        this->get_parameter("rate_hz", rate_);

        pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/vision/target", 10);
        timer_ = this->create_wall_timer(
          std::chrono::duration<double>(1.0 / rate_),
          std::bind(&DummyTargetPublisher::tick_, this));
        RCLCPP_INFO(get_logger(), "dummy_target_publisher running @ %.1f Hz", rate_);
    }

private:
    void tick_() {
        auto now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = parent_frame_;
        t.child_frame_id = child_frame_;

        const double s = std::sin(now.seconds());
        t.transform.translation.x = 0.10 * s;
        t.transform.translation.y = 0.00;
        t.transform.translation.z = 0.40;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_.sendTransform(t);
        pub_->publish(t);
    }

    std::string parent_frame_;
    std::string child_frame_;
    double rate_{30.0};

    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyTargetPublisher>());
    rclcpp::shutdown();
    return 0;
}
