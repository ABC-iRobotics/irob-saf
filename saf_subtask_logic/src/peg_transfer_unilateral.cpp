#include <saf_subtask_logic/peg_transfer_unilateral.hpp>
#include <chrono>
#include <thread>
#include <iostream>

namespace saf {

PegTransferUnilateral::PegTransferUnilateral(
    const rclcpp::NodeOptions &options,
    const std::vector<std::string> &arm_names)
: PegTransferLogic(options, arm_names), arm_names_(arm_names)
{
  declare_and_get_params_();
  RCLCPP_INFO(this->get_logger(),
              "[PT uni] board='%s' offset1='%s' arms=[%s] thr=%.3f",
              board_descriptor_file_.c_str(),
              offset_file_arm_1_.c_str(),
              arm_names_.empty() ? "" : arm_names_[0].c_str(),
              on_dist_threshold_);
}

void PegTransferUnilateral::declare_and_get_params_() {
  this->declare_parameter<std::vector<std::string>>("arm_names", arm_names_);
  this->declare_parameter<std::string>("board_descriptor_file", "peg_transfer_board.yaml");
  this->declare_parameter<std::string>("offset_file_arm_1", "offset_psm_2_uni.yaml");
  this->declare_parameter<double>("on_dist_threshold", on_dist_threshold_);
  this->declare_parameter<std::string>("mode", "execution");

  (void)this->get_parameter("arm_names", arm_names_);
  (void)this->get_parameter("board_descriptor_file", board_descriptor_file_);
  (void)this->get_parameter("offset_file_arm_1", offset_file_arm_1_);
  (void)this->get_parameter("on_dist_threshold", on_dist_threshold_);
  std::string mode_s; (void)this->get_parameter("mode", mode_s); mode_ = mode_from_string_(mode_s);

  // If PegTransferLogic needs these, set them there too (helpers you already have):
  set_board_descriptor(board_descriptor_file_);
  set_offset_file_arm1(offset_file_arm_1_);
  set_on_distance_threshold(on_dist_threshold_);
}

PegTransferUnilateral::Mode PegTransferUnilateral::mode_from_string_(const std::string &s) {
    if (s == "execution")   return Mode::EXECUTION;
    if (s == "calibration") return Mode::CALIBRATION;
    if (s == "acc_blocks")  return Mode::ACC_BLOCKS;
    if (s == "acc_pegs")    return Mode::ACC_PEGS;
    RCLCPP_WARN(rclcpp::get_logger("peg_transfer_unilateral"),
                "Unknown mode '%s', defaulting to execution", s.c_str());
    return Mode::EXECUTION;
}

void PegTransferUnilateral::run() {
  switch (mode_) {
    case Mode::EXECUTION:   doPegTransfer();         break;
    case Mode::CALIBRATION: calibrateOffset();       break;
    case Mode::ACC_BLOCKS:  measureAccuracyBlocks(); break;
    case Mode::ACC_PEGS:    measureAccuracyPegs();   break;
  }
}

// ===== the following four should contain your original bodies =====
//     (calls to vision, storeEnvironment, arms[...], file I/O, etc.)

void PegTransferUnilateral::doPegTransfer() {
  RCLCPP_INFO(this->get_logger(), "[PT uni] EXECUTION start");
  // TODO: paste ROS1 logic here 1:1; ROS APIs already replaced in base
  RCLCPP_INFO(this->get_logger(), "[PT uni] EXECUTION done");
}

void PegTransferUnilateral::calibrateOffset() {
  RCLCPP_INFO(this->get_logger(), "[PT uni] CALIBRATION start");
  // TODO: paste ROS1 logic here (compute/save offset_file_arm_1_)
  RCLCPP_INFO(this->get_logger(), "[PT uni] CALIBRATION done → '%s'", offset_file_arm_1_.c_str());
}

void PegTransferUnilateral::measureAccuracyBlocks() {
  RCLCPP_INFO(this->get_logger(), "[PT uni] ACC_BLOCKS start");
  // TODO: paste ROS1 logic here
  RCLCPP_INFO(this->get_logger(), "[PT uni] ACC_BLOCKS done");
}

void PegTransferUnilateral::measureAccuracyPegs() {
  RCLCPP_INFO(this->get_logger(), "[PT uni] ACC_PEGS start");
  // TODO: paste ROS1 logic here
  RCLCPP_INFO(this->get_logger(), "[PT uni] ACC_PEGS done");
}

} // namespace saf

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<saf::PegTransferUnilateral>();
  node->run();                 // run once; no need to spin forever
  rclcpp::shutdown();
  return 0;
}
