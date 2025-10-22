#include <saf_subtask_logic/grasp.hpp>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

namespace saf {

    Grasp::Grasp(const rclcpp::NodeOptions &options,
             const std::vector<std::string> &arm_names)
        : AutosurgAgent("dummy_grasp", options),
              vision_(this->shared_from_this(), "/vision/target"),   // ✅ correct namespace
              arm_names_(arm_names)
    {
        this->declare_parameter<std::vector<std::string>>("arm_names", arm_names_);
        this->get_parameter("arm_names", arm_names_);

        if (arms().empty()) {
            for (const auto &name : arm_names_)
                arms().push_back(std::make_shared<saf::SurgemeClient>(this->shared_from_this(), name));
        }

        RCLCPP_INFO(get_logger(), "Grasp node ready, arm=%s",
                    arm_names_.empty() ? "none" : arm_names_[0].c_str());
    }

    void Grasp::graspObject()
    {
        std::optional<Eigen::Affine3d> maybe_pose;
        while (rclcpp::ok()) {
            maybe_pose = vision_.latest();
            if (maybe_pose) break;
            std::this_thread::sleep_for(100ms);
        }
        if (!maybe_pose) {
            RCLCPP_WARN(get_logger(), "No target received from vision");
            return;
        }
        Eigen::Affine3d p = *maybe_pose;

        const double approach_dist = 0.01;  // 10 mm
        Eigen::Affine3d approach_pose = Eigen::Translation3d(0.0, 0.0, -approach_dist) * p;

        auto arm = arms().at(0);
        RCLCPP_INFO(get_logger(), "Executing grasp sequence");
        arm->grasp();  // simplified call
        RCLCPP_INFO(get_logger(), "Grasp command sent");
    }

}  // namespace saf

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<saf::Grasp>();
    node->graspObject();
    rclcpp::shutdown();
    return 0;
}
