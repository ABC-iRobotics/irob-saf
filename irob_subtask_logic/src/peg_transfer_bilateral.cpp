#include <irob_subtask_logic/peg_transfer_bilateral.hpp>
#include <chrono>
#include <thread>
#include <iostream>

namespace saf {

PegTransferBilateral::PegTransferBilateral(
    const rclcpp::NodeOptions &options,
    const std::vector<std::string> &arm_names)
: PegTransferLogic(options, arm_names), arm_names_(arm_names)
{
  declare_and_get_params_();
  RCLCPP_INFO(this->get_logger(),
              "[PT bi] board='%s' off1='%s' off2='%s' arms=[%s,%s] thr=%.3f",
              board_descriptor_file_.c_str(),
              offset_file_arm_1_.c_str(),
              offset_file_arm_2_.c_str(),
              arm_names_.size()>0?arm_names_[0].c_str():"",
              arm_names_.size()>1?arm_names_[1].c_str():"",
              on_dist_threshold_);
}

void PegTransferBilateral::declare_and_get_params_() {
  this->declare_parameter<std::vector<std::string>>("arm_names", arm_names_);
  this->declare_parameter<std::string>("board_descriptor_file", "peg_transfer_board.yaml");
  this->declare_parameter<std::string>("offset_file_arm_1", "offset_psm1.yaml");
  this->declare_parameter<std::string>("offset_file_arm_2", "offset_psm2.yaml");
  this->declare_parameter<double>("on_dist_threshold", on_dist_threshold_);
  this->declare_parameter<std::string>("mode", "execution");

  (void)this->get_parameter("arm_names", arm_names_);
  (void)this->get_parameter("board_descriptor_file", board_descriptor_file_);
  (void)this->get_parameter("offset_file_arm_1", offset_file_arm_1_);
  (void)this->get_parameter("offset_file_arm_2", offset_file_arm_2_);
  (void)this->get_parameter("on_dist_threshold", on_dist_threshold_);
  std::string mode_s; (void)this->get_parameter("mode", mode_s); mode_ = mode_from_string_(mode_s);

  // push into base
  set_board_descriptor(board_descriptor_file_);
  set_offset_file_arm1(offset_file_arm_1_);
  set_offset_file_arm2(offset_file_arm_2_);
  set_on_distance_threshold(on_dist_threshold_);
}

PegTransferBilateral::Mode PegTransferBilateral::mode_from_string_(const std::string &s) {
    if (s == "execution")   return Mode::EXECUTION;
    if (s == "calibration") return Mode::CALIBRATION;
    if (s == "acc_blocks")  return Mode::ACC_BLOCKS;
    if (s == "acc_pegs")    return Mode::ACC_PEGS;
    RCLCPP_WARN(rclcpp::get_logger("peg_transfer_bilateral"),
                "Unknown mode '%s', defaulting to execution", s.c_str());
    return Mode::EXECUTION;
}


void PegTransferBilateral::run() {
  switch (mode_) {
    case Mode::EXECUTION:   doPegTransfer();         break;
    case Mode::CALIBRATION: calibrateOffset();       break;
    case Mode::ACC_BLOCKS:  measureAccuracyBlocks(); break;
    case Mode::ACC_PEGS:    measureAccuracyPegs();   break;
  }
}

// ===== paste your ROS1 bodies for bilateral here =====

void PegTransferBilateral::doPegTransfer() {
  RCLCPP_INFO(this->get_logger(), "[PT bi] EXECUTION start");
  // TODO: paste ROS1 logic (uses vision, storeEnvironment, arms[...])
  RCLCPP_INFO(this->get_logger(), "[PT bi] EXECUTION done");
}

void PegTransferBilateral::calibrateOffset() {
  RCLCPP_INFO(this->get_logger(), "[PT bi] CALIBRATION start");
  // TODO: paste ROS1 logic (compute & save both offsets)
  RCLCPP_INFO(this->get_logger(), "[PT bi] CALIBRATION done → '%s' & '%s'",
              offset_file_arm_1_.c_str(), offset_file_arm_2_.c_str());
}

void PegTransferBilateral::measureAccuracyBlocks() {
  RCLCPP_INFO(this->get_logger(), "[PT bi] ACC_BLOCKS start");
  // TODO: paste ROS1 logic
  RCLCPP_INFO(this->get_logger(), "[PT bi] ACC_BLOCKS done");
}

void PegTransferBilateral::measureAccuracyPegs() {
  RCLCPP_INFO(this->get_logger(), "[PT bi] ACC_PEGS start");
  // TODO: paste ROS1 logic
  RCLCPP_INFO(this->get_logger(), "[PT bi] ACC_PEGS done");
}

} // namespace saf

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<saf::PegTransferBilateral>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
