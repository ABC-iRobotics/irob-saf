/*
 *  robot_server_psm.cpp
 *
 *  Author(s): Tamas Levendovics
 *  Created on: 2016-11-07
 *
 */

#include <irob_dvrk/robot_server_psm.hpp>
#include <numeric>
#include <chrono>
#include <irob_utils/trajectory_factory.hpp>

namespace saf {

/*
 * Constructor
 */
RobotServerPSM::RobotServerPSM(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr priv_node,
                               ArmTypes arm_typ, std::string arm_name,
                               bool isActive) :
  RobotServerDVRK(node, priv_node, arm_typ, arm_name, isActive)
{
  if (!(arm_typ == ArmTypes::PSM1 ||
        arm_typ == ArmTypes::PSM2 ||
        arm_typ == ArmTypes::PSM3))
    throw std::runtime_error(
        "Tried to create RobotServerPSM object for ECM or MTM arm type.");
}

/*
 * irob_msgs/Robot actions
 */

void RobotServerPSM::resetPose(bool move_allowed)
{
  // helper variables
  bool success = false;

  irob_msgs::msg::RobotFeedback feedback;
  irob_msgs::msg::RobotResult result;

  RCLCPP_INFO(this->get_logger(), "Starting %s pose reset", arm_typ.name.c_str());

  // Check that preempt has not been requested by the client
  if (as.is_preempt_requested() || !rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(), "%s pose reset: Preempted", arm_typ.name.c_str());
    // Set the action state to preempted
    as.set_preempted();
    success = false;
  }

  RCLCPP_INFO(this->get_logger(), "%s pose reset not implemented", arm_typ.name.c_str());
  success = true;
  // Send some feedback
  feedback.info = "done";
  result.pose = getPoseCurrent().toRosToolPose();
  as.publish_feedback(feedback);

  if(success)
  {
    result.info = "done";
    result.pose = getPoseCurrent().toRosToolPose();
    RCLCPP_INFO(this->get_logger(), "%s pose reset succeeded", arm_typ.name.c_str());
    // set the action state to succeeded
    as.set_succeeded(result);
  }
}

RobotServerPSM::~RobotServerPSM()
{
  // TODO Auto-generated destructor stub
}

void RobotServerPSM::subscribeLowLevelTopics()
{
  RobotServerDVRK::subscribeLowLevelTopics();

  status_sub = this->create_subscription<sensor_msgs::msg::JointState>(
        TopicNameLoader::load(this, arm_typ.name, "dvrk_topics/state_jaw_current"),
        1000, std::bind(&RobotServerPSM::positionJawCurrentCB, this, std::placeholders::_1));
}

void RobotServerPSM::advertiseLowLevelTopics()
{
  RobotServerDVRK::advertiseLowLevelTopics();

  position_jaw_pub = this->create_publisher<sensor_msgs::msg::JointState>(
        TopicNameLoader::load(this, arm_typ.name, "dvrk_topics/set_position_jaw"),
        1000);
}

/*
 * Callbacks
 */

void RobotServerPSM::measured_cp_cb(
    const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  measured_cp = *msg;
  irob_msgs::msg::ToolPoseStamped fwd;
  fwd.header = measured_cp.header;
  while (jaw_measured_js.position.empty())
  {
    rclcpp::spin_some(this->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ToolPose tmp(measured_cp, jaw_measured_js.position[0]);

  tmp = T_he.inverse() * tmp;
  // Hand-eye calibration
  // Convert from m-s to mm-s
  fwd.toolpose = tmp.toRosToolPose();
  measured_cp_pub->publish(fwd);
}

void RobotServerPSM::positionJawCurrentCB(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  jaw_measured_js = *msg;
}

ToolPose RobotServerPSM::getPoseCurrent()
{
  rclcpp::spin_some(this->get_node_base_interface());
  ToolPose ret(measured_cp, jaw_measured_js.position[0]);
  return ret;
}

/*
 * DVRK actions
 */
void RobotServerPSM::moveJawRelative(double movement, double dt)
{
  // Collect data
  ToolPose currPose = getPoseCurrent();
  ToolPose pose = currPose;
  pose.jaw += movement;
  sensor_msgs::msg::JointState new_position_jaw
      = wrapToMsg<sensor_msgs::msg::JointState,double>(pose.jaw);
  try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety

    // Publish movement
    position_jaw_pub->publish(new_position_jaw);
    rclcpp::spin_some(this->get_node_base_interface());
  } catch (std::runtime_error& e)
  {
    std::string str_nan = "NaN";
    std::string str_err = e.what();
    if (str_err.find(str_nan) != std::string::npos)
      RCLCPP_ERROR(this->get_logger(), e.what());
    else
      throw e;
  }
}

/*
 * Move the grippers immediately.
 */
void RobotServerPSM::moveJawAbsolute(double jaw, double dt)
{
  // Collect data
  ToolPose currPose = getPoseCurrent();
  ToolPose pose = currPose;
  pose.jaw = jaw;
  sensor_msgs::msg::JointState new_position_jaw
      = wrapToMsg<sensor_msgs::msg::JointState,double>(pose.jaw);
  try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety

    // Publish movement
    position_jaw_pub->publish(new_position_jaw);
    rclcpp::spin_some(this->get_node_base_interface());
  } catch (std::runtime_error& e)
  {
    std::string str_nan = "NaN";
    std::string str_err = e.what();
    if (str_err.find(str_nan) != std::string::npos)
      RCLCPP_ERROR(this->get_logger(), e.what());
    else
      throw e;
  }
}

/*
 * Move in cartesian, and move jaw immediately.
 */
void RobotServerPSM::moveCartesianAbsolute(ToolPose pose, double dt)
{
  // Collect data
  ToolPose currPose = getPoseCurrent();
  geometry_msgs::msg::Transform new_position_cartesian
      = wrapToMsg<geometry_msgs::msg::Transform, Eigen::Affine3d>(pose.transform);
  geometry_msgs::msg::TransformStamped new_position_cartesian_stamped(measured_cp);
  new_position_cartesian_stamped.transform=new_position_cartesian;
  sensor_msgs::msg::JointState new_position_jaw
      = wrapToMsg<sensor_msgs::msg::JointState,double>(pose.jaw);

  try{
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety

    // Publish movement
    position_cartesian_pub->publish(new_position_cartesian_stamped);
    position_jaw_pub->publish(new_position_jaw);
    rclcpp::spin_some(this->get_node_base_interface());
  } catch (std::runtime_error& e)
  {
    std::string str_nan = "NaN";
    std::string str_err = e.what();
    if (str_err.find(str_nan) != std::string::npos)
      RCLCPP_ERROR(this->get_logger(), e.what());
    else
      throw e;
  }
}

}

using namespace saf;

/**
 * Main for RobotServerPSM
 */
int main(int argc, char **argv)
{
  // Initialize ros2 node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("robot_server_dvrk");
  rclcpp::Node::SharedPtr priv_nh = rclcpp::Node::make_shared("~");

  std::string arm_typ;
  priv_nh->get_parameter("arm_typ", arm_typ);
  ArmTypes arm_type = ArmTypes::typeForString(arm_typ);

  std::string arm_name;
  priv_nh->get_parameter("arm_name", arm_name);

  std::string filename;
  priv_nh->get_parameter("filename", filename);

  // Robot control
  try {
    if (arm_type == ArmTypes::PSM1 || arm_type == ArmTypes::PSM2 || arm_type == ArmTypes::PSM3) {
      RobotServerPSM psm(nh, priv_nh, arm_type,
                         arm_name, RobotServerPSM::ACTIVE);
      psm.initRosCommunication();
      rclcpp::spin(nh);
      //psm.saveTrajectory(filename);
    }
    else {
      RobotServerDVRK arm(nh, priv_nh, arm_type,
                          arm_name, RobotServerDVRK::ACTIVE);
      arm.initRosCommunication();
      rclcpp::spin(nh);
      //arm.saveTrajectory(filename);
    }

    RCLCPP_INFO(nh->get_logger(), "Program finished successfully, shutting down ...");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(nh->get_logger(), e.what());
    RCLCPP_ERROR(nh->get_logger(), "Program stopped by an error, shutting down ...");
  }

  // Exit
  rclcpp::shutdown();
  return 0;
}
