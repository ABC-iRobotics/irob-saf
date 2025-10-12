#pragma once
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

/**
 * A trivial image "processor" that fabricates a moving target transform.
 * Replace this with your real model-based or ML-based vision implementation.
 */
namespace irob_vision {

    class DummyImageProcessor {
    public:
        DummyImageProcessor() = default;

        geometry_msgs::msg::TransformStamped compute(const sensor_msgs::msg::Image &/*img*/,
                                                     const std::string &parent_frame = "camera_frame",
                                                     const std::string &child_frame = "target_frame",
                                                     rclcpp::Time stamp = rclcpp::Clock().now()) const
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp;
            t.header.frame_id = parent_frame;
            t.child_frame_id = child_frame;

            // simple oscillation in x
            const double s = std::sin(stamp.seconds());
            t.transform.translation.x = 0.10 * s; // meters
            t.transform.translation.y = 0.00;
            t.transform.translation.z = 0.40;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;
            return t;
        }
    };

} // namespace irob_vision
