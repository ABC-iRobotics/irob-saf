#pragma once

#include <string>
#include <vector>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <saf_subtask_logic/autosurg_agent.hpp>

namespace saf {

/**
 * @brief Abstract base for peg-transfer logic.
 *        Keeps shared configuration (board/offsets/threshold), environment,
 *        and simple accessors used by unilateral/bilateral modes.
 */
class PegTransferLogic : public AutosurgAgent {
public:
  // A minimal environment holder; adapt this to match your real vision output.
  struct Environment {
    // TODO: populate with what your vision module returns (peg centers, block poses, etc.)
    // e.g., std::vector<Eigen::Isometry3d> pegs;
    //       std::vector<Eigen::Isometry3d> blocks;
    //       Eigen::Isometry3d board_pose;
  };

  explicit PegTransferLogic(
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
      const std::vector<std::string> & arm_names = {})
  : AutosurgAgent("peg_transfer_logic", options),
    arm_names_(arm_names)
  {
    // Shared params that children often rely on
    this->declare_parameter<std::string>("board_descriptor_file", board_descriptor_file_);
    this->declare_parameter<std::string>("offset_file_arm_1", offset_file_arm_1_);
    this->declare_parameter<std::string>("offset_file_arm_2", offset_file_arm_2_);
    this->declare_parameter<double>("on_dist_threshold", on_dist_threshold_);

    // Pull initial values if provided
    (void)this->get_parameter("board_descriptor_file", board_descriptor_file_);
    (void)this->get_parameter("offset_file_arm_1", offset_file_arm_1_);
    (void)this->get_parameter("offset_file_arm_2", offset_file_arm_2_);
    (void)this->get_parameter("on_dist_threshold", on_dist_threshold_);
  }

  virtual ~PegTransferLogic() = default;

  // --------------------------------------------------------------------------
  // Configuration setters/getters (used from unilateral/bilateral nodes)
  // --------------------------------------------------------------------------
  void set_board_descriptor(std::string file) { board_descriptor_file_ = std::move(file); }
  void set_offset_file_arm1(std::string file) { offset_file_arm_1_    = std::move(file); }
  void set_offset_file_arm2(std::string file) { offset_file_arm_2_    = std::move(file); }
  void set_on_distance_threshold(double d)    { on_dist_threshold_    = d; }

  const std::string& board_descriptor_file() const { return board_descriptor_file_; }
  const std::string& offset_file_arm_1() const     { return offset_file_arm_1_; }
  const std::string& offset_file_arm_2() const     { return offset_file_arm_2_; }
  double on_distance_threshold() const             { return on_dist_threshold_; }

  const std::vector<std::string>& arm_names() const { return arm_names_; }

  // --------------------------------------------------------------------------
  // Vision / environment handoff
  // --------------------------------------------------------------------------
  void storeEnvironment(const Environment& e) { env_ = e; }
  const std::optional<Environment>& environment() const { return env_; }
  std::optional<Environment>& environment() { return env_; }

  // If you want a default no-op vision fetcher; override in derived or wire your vision module.
  virtual bool fetchEnvironmentFromVision() {
    // TODO: call your real vision provider and then storeEnvironment(...)
    // Keep a no-op default so abstract class compiles.
    RCLCPP_WARN(this->get_logger(), "fetchEnvironmentFromVision() is not implemented");
    return false;
  }

  // Utility: simple “are we close enough” check (uses threshold param)
  static bool within_threshold_mm(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double thr_mm) {
    return (a - b).norm() * 1000.0 <= thr_mm; // assume 'a','b' are in meters; convert to mm
  }

protected:
  // Names of the arms involved (logical names, e.g. "arm_1","arm_2")
  std::vector<std::string> arm_names_;

  // Shared config
  std::string board_descriptor_file_{"peg_transfer_board.yaml"};
  std::string offset_file_arm_1_{"offset_psm1.yaml"};
  std::string offset_file_arm_2_{"offset_psm2.yaml"};
  double on_dist_threshold_{5.0}; // mm

  // Last perceived environment
  std::optional<Environment> env_;
};

} // namespace saf
