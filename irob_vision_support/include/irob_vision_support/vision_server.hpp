#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <string>

namespace irob_vision {

/**
 * Generic vision server node that:
 *  - subscribes to (preprocessed) images,
 *  - uses a processor functor/class with compute(img)->TransformStamped,
 *  - publishes the target as TF and on a topic.
 *
 * Processor must have:
 *   geometry_msgs::msg::TransformStamped compute(const sensor_msgs::msg::Image&, const std::string&, const std::string&, rclcpp::Time)
 */
template <class ProcessorT>
class VisionServer : public rclcpp::Node {
public:
  explicit VisionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : rclcpp::Node("vision_server", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    this->declare_parameter<std::string>("image_topic", image_topic_);
    this->declare_parameter<std::string>("target_topic", target_topic_);
    this->declare_parameter<std::string>("parent_frame", parent_frame_);
    this->declare_parameter<std::string>("child_frame", child_frame_);
    this->get_parameter("image_topic", image_topic_);
    this->get_parameter("target_topic", target_topic_);
    this->get_parameter("parent_frame", parent_frame_);
    this->get_parameter("child_frame", child_frame_);

    pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(target_topic_, 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&VisionServer::img_cb_, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "vision_server: img='%s' → target='%s' (%s->%s)",
                image_topic_.c_str(), target_topic_.c_str(),
                parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void img_cb_(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto stamp = this->get_clock()->now();
    const auto t = proc_.compute(*msg, parent_frame_, child_frame_, stamp);

    // publish as TF
    tf_broadcaster_.sendTransform(t);
    // and as a topic message
    pub_->publish(t);
  }

  // params
  std::string image_topic_{"/camera/image_preprocessed"};
  std::string target_topic_{"/vision/target"};
  std::string parent_frame_{"camera_frame"};
  std::string child_frame_{"target_frame"};

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // vision processor
  ProcessorT proc_;
};

} // namespace irob_vision
