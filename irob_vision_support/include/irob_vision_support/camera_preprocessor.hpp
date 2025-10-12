#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace irob_vision {

    /**
     * Minimal pass-through camera preprocessor.
     * Subscribes to raw image and republishes preprocessed image (identity for now).
     * Extend with CV ops later (resize/blur/etc.).
     */
    class CameraPreprocessor : public rclcpp::Node {
    public:
        explicit CameraPreprocessor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

        std::string in_topic_{"/camera/image_raw"};
        std::string out_topic_{"/camera/image_preprocessed"};

        void image_cb_(const sensor_msgs::msg::Image::SharedPtr msg);
    };

} // namespace irob_vision
