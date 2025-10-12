#pragma once
/*
 *  grasp.hpp (ROS 2)
 */

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <string>

#include <irob_subtask_logic/autosurg_agent.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_vision_support/vision_client.hpp>   // ✅ correct include

namespace saf {

    /**
     * @brief Autonomous grasp sub-task node.
     */
    class Grasp : public AutosurgAgent {
    public:
        explicit Grasp(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
            const std::vector<std::string> &arm_names = {"arm_1"});

        ~Grasp() override = default;

        void graspObject();

    private:
        // ✅ Correct namespace and member declaration
        irob_vision::VisionClient vision_;

        std::vector<std::string> arm_names_;
    };

} // namespace saf
