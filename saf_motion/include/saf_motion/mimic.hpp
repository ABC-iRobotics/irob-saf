#pragma once
/*
 *  mimic.hpp (ROS 2)
 *  Node that "mimes" / mirrors motions; kept from ROS1 concept.
 */

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace saf {

    class Mimic : public rclcpp::Node {
    public:
        static constexpr double DEFAULT_LOOP_RATE = 10.0; // Hz
        explicit Mimic(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        void spin_loop();

    private:
        double loop_rate_hz_;
        std::string arm_in_;
        std::string arm_out_;
        void declare_and_get_params_();
    };

} // namespace saf
