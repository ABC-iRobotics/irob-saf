#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <saf_subtask_logic/peg_transfer_logic.hpp>

namespace saf {

    class PegTransferUnilateral : public PegTransferLogic {
    public:
        enum class Mode { EXECUTION, CALIBRATION, ACC_BLOCKS, ACC_PEGS };

        explicit PegTransferUnilateral(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
            const std::vector<std::string> &arm_names = {"arm_2"});

        void run();                      // runs once based on 'mode' param
        void set_mode(Mode m) { mode_ = m; }

    private:
        // params
        std::vector<std::string> arm_names_;
        std::string board_descriptor_file_;
        std::string offset_file_arm_1_;
        double on_dist_threshold_{5.0};
        Mode mode_{Mode::EXECUTION};

        void declare_and_get_params_();
        static Mode mode_from_string_(const std::string &s);

        // same high-level ops as ROS1
        void doPegTransfer();
        void calibrateOffset();
        void measureAccuracyBlocks();
        void measureAccuracyPegs();
    };

} // namespace saf
