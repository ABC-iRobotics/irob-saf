#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// If you use your surgeme client helper, include it here:
#include <irob_motion/surgeme_client.hpp>   // class saf::SurgemeClient

namespace saf {

/**
 * @brief Base abstract agent node for AutoSurg components (ROS 2).
 *        Replaces ros::NodeHandle with rclcpp::Node. Provides TF2 helpers
 *        and a place to store per-arm control clients.
 */
class AutosurgAgent : public rclcpp::Node {
public:
  using Clock = rclcpp::Clock;

  explicit AutosurgAgent(
      const std::string & node_name,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    // Common parameters can be declared here if you had any global ones.
    // Example:
    // this->declare_parameter<bool>("use_sim_time", false);
  }

  virtual ~AutosurgAgent() = default;

  // ---- Convenience helpers -------------------------------------------------

  rclcpp::Time now() const { return this->get_clock()->now(); }

  template <typename Rep, typename Period>
  static void sleep_for(std::chrono::duration<Rep, Period> d) {
    std::this_thread::sleep_for(d);
  }

  static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

  // ---- TF2 utilities -------------------------------------------------------

  tf2_ros::Buffer& tf_buffer() { return tf_buffer_; }
  const tf2_ros::Buffer& tf_buffer() const { return tf_buffer_; }

  tf2_ros::TransformBroadcaster& tf_broadcaster() { return tf_broadcaster_; }

  // ---- Arms interface (clients to motion/surgeme, or your own wrappers) ----
  // Populate from your derived class or a setup method.
  std::vector<std::shared_ptr<saf::SurgemeClient>>& arms() { return arms_; }
  const std::vector<std::shared_ptr<saf::SurgemeClient>>& arms() const { return arms_; }

protected:
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Your per-arm control clients (fill in derived constructors)
  std::vector<std::shared_ptr<saf::SurgemeClient>> arms_;

  // You can add common protected helpers here (e.g., Pose conversions, logging wrappers, etc.)
};

} // namespace saf
