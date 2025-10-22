#include <saf_motion/mimic.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace saf {

    Mimic::Mimic(const rclcpp::NodeOptions& options)
      : rclcpp::Node("mimic", options)
    {
        declare_and_get_params_();
        RCLCPP_INFO(get_logger(), "mimic running: in='%s' out='%s' loop=%.1f Hz",
                    arm_in_.c_str(), arm_out_.c_str(), loop_rate_hz_);
    }

    void Mimic::declare_and_get_params_() {
        this->declare_parameter<double>("loop_rate_hz", DEFAULT_LOOP_RATE);
        this->declare_parameter<std::string>("arm_in", "psm1");
        this->declare_parameter<std::string>("arm_out", "psm2");
        this->get_parameter("loop_rate_hz", loop_rate_hz_);
        this->get_parameter("arm_in", arm_in_);
        this->get_parameter("arm_out", arm_out_);
    }

    void Mimic::spin_loop() {
        rclcpp::Rate rate(loop_rate_hz_);
        while (rclcpp::ok()) {
            // TODO: read from arm_in_ topics and publish to arm_out_ topics.
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
    }

} // namespace saf

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<saf::Mimic>();
    node->spin_loop();
    rclcpp::shutdown();
    return 0;
}
