#include <saf_vision/camera_preprocessor.hpp>

namespace saf {

    CameraPreprocessor::CameraPreprocessor(const rclcpp::NodeOptions &options)
    : rclcpp::Node("camera_preprocessor", options)
    {
        this->declare_parameter<std::string>("in_topic", in_topic_);
        this->declare_parameter<std::string>("out_topic", out_topic_);
        this->get_parameter("in_topic", in_topic_);
        this->get_parameter("out_topic", out_topic_);

        pub_ = this->create_publisher<sensor_msgs::msg::Image>(out_topic_, rclcpp::SensorDataQoS());
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            in_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CameraPreprocessor::image_cb_, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "camera_preprocessor: '%s' -> '%s'",
                    in_topic_.c_str(), out_topic_.c_str());
    }

    void CameraPreprocessor::image_cb_(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // TODO: add actual preprocessing; for now just republish.
        pub_->publish(*msg);
    }

} // namespace saf

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<saf::CameraPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
