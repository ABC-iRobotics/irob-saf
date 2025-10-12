#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>
#include <mutex>
#include <optional>

namespace irob_vision {

/**
 * Minimal vision client helper:
 *   - subscribes to TransformStamped on a topic (default: /vision/target)
 *   - stores the latest transform
 *   - getResult() returns Eigen::Affine3d; throws if none received yet (optional)
 */
class VisionClient {
public:
  VisionClient(const rclcpp::Node::SharedPtr& node,
               const std::string& target_topic = "/vision/target")
  : node_(node)
  {
    sub_ = node_->create_subscription<geometry_msgs::msg::TransformStamped>(
        target_topic, 10,
        std::bind(&VisionClient::cb_, this, std::placeholders::_1));
  }

  // Blocking poll: wait up to 'timeout' for first result, then return latest pose
  std::optional<Eigen::Affine3d> waitForResult(std::chrono::milliseconds timeout) {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout && rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lk(m_);
        if (last_) return to_eigen_(*last_);
      }
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return std::nullopt;
  }

  // Return latest (if any)
  std::optional<Eigen::Affine3d> latest() const {
    std::lock_guard<std::mutex> lk(m_);
    if (!last_) return std::nullopt;
    return to_eigen_(*last_);
  }

private:
  void cb_(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(m_);
    last_ = *msg;
  }

  static Eigen::Affine3d to_eigen_(const geometry_msgs::msg::TransformStamped& t) {
    Eigen::Translation3d trans(t.transform.translation.x,
                               t.transform.translation.y,
                               t.transform.translation.z);
    Eigen::Quaterniond q(t.transform.rotation.w,
                         t.transform.rotation.x,
                         t.transform.rotation.y,
                         t.transform.rotation.z);
    return trans * q;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_;
  mutable std::mutex m_;
  std::optional<geometry_msgs::msg::TransformStamped> last_;
};

} // namespace irob_vision
