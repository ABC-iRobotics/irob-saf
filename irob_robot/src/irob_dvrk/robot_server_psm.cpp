/*
 *  robot_server_psm.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
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
RobotServerPSM::RobotServerPSM(ros::NodeHandle nh,ros::NodeHandle priv_nh,
                               ArmTypes arm_typ, std::string arm_name,
                               bool isActive):
  RobotServerDVRK(nh, priv_nh, arm_typ, arm_name, isActive)
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

  irob_msgs::RobotFeedback feedback;
  irob_msgs::RobotResult result;

  ROS_INFO_STREAM("Starting " << arm_typ.name << " pose reset");
  
  // Check that preempt has not been requested by the client
  if (as.isPreemptRequested() || !ros::ok())
  {
    ROS_INFO_STREAM(arm_typ.name << " pose reset: Preempted");
    // Set the action state to preempted
    as.setPreempted();
    success = false;
  }

  ROS_INFO_STREAM(arm_typ.name << " pose reset not implemented");
  success = true;
  // Send some feedback
  feedback.info = "done";
  result.pose = getPoseCurrent().toRosToolPose();
  as.publishFeedback(feedback);

  if(success)
  {
    result.info = "done";
    result.pose = getPoseCurrent().toRosToolPose();
    ROS_INFO_STREAM(arm_typ.name << " pose reset succeded");
    // set the action state to succeeded
    as.setSucceeded(result);
  }
}

RobotServerPSM::~RobotServerPSM()
{
  // TODO Auto-generated destructor stub
}

void RobotServerPSM::subscribeLowLevelTopics()
{
  RobotServerDVRK::subscribeLowLevelTopics();

  current_state_sub = nh.subscribe<sensor_msgs::JointState>(
        TopicNameLoader::load(priv_nh,
                              "dvrk_topics/namespace",
                              arm_typ.name,
                              "dvrk_topics/state_jaw_current"),
        1000, &RobotServerPSM::positionJawCurrentCB,this);
}

void RobotServerPSM::advertiseLowLevelTopics()
{
  RobotServerDVRK::advertiseLowLevelTopics();

  position_jaw_pub = nh.advertise<sensor_msgs::JointState>(
        TopicNameLoader::load(priv_nh,
                              "dvrk_topics/namespace",
                              arm_typ.name,
                              "dvrk_topics/set_position_jaw"),
        1000);
}

/*
 * Callbacks
 */

void RobotServerPSM::positionCartesianCurrentCB(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
  position_cartesian_current = *msg;
  irob_msgs::ToolPoseStamped fwd;
  fwd.header = position_cartesian_current.header;
  while (position_jaw.position.empty())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();

  }
  Pose tmp(position_cartesian_current, position_jaw.position[0]);
  // Hand-eye calibration
  // Convert from m-s to mm-s
  fwd.pose = tmp.invTransform(R, t, 0.001).toRosToolPose();
  position_cartesian_current_pub.publish(fwd);
}

void RobotServerPSM::positionJawCurrentCB(
    const sensor_msgs::JointStateConstPtr& msg)
{
  position_jaw = *msg;
}

Pose RobotServerPSM::getPoseCurrent()
{
  ros::spinOnce();
  Pose ret(position_cartesian_current, position_jaw.position[0]);
  return ret;
}

/*
 * DVRK actions
 */
void RobotServerPSM::moveJawRelative(double movement, double dt)
{
  // Collect data
  Pose currPose = getPoseCurrent();
  Pose pose = currPose;
  pose.jaw += movement;
  sensor_msgs::JointState new_position_jaw = pose.toRosJaw();
  try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
  } catch (std::runtime_error e)
  {
    std::string str_nan = "NaN";
    std::string str_err = e.what();
    if (str_err.find(str_nan) != std::string::npos)
      ROS_ERROR_STREAM(e.what());
    else
      throw e;
  }
}

/*
 * Move the grippers immediatley.
 */
void RobotServerPSM::moveJawAbsolute(double jaw, double dt)
{
  // Collect data
  Pose currPose = getPoseCurrent();
  Pose pose = currPose;
  pose.jaw = jaw;
  sensor_msgs::JointState new_position_jaw = pose.toRosJaw();
  try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
  } catch (std::runtime_error e)
  {
    std::string str_nan = "NaN";
    std::string str_err = e.what();
    if (str_err.find(str_nan) != std::string::npos)
      ROS_ERROR_STREAM(e.what());
    else
      throw e;
  }
}

/*
 * Move in cartesian, and move jaw immediately.
 */
void RobotServerPSM::moveCartesianAbsolute(Pose pose, double dt)
{
  // Collect data
  Pose currPose = getPoseCurrent();
  geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
  sensor_msgs::JointState new_position_jaw = pose.toRosJaw();
  try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
  } catch (std::runtime_error e)
  {
    std::string str_nan = "NaN";
    std::string str_err = e.what();
    if (str_err.find(str_nan) != std::string::npos)
      ROS_ERROR_STREAM(e.what());
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

  // Initialize ros node
  ros::init(argc, argv, "robot_server_dvrk");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string arm_typ;
  priv_nh.getParam("arm_typ", arm_typ);
  ArmTypes arm_type = ArmTypes::typeForString(arm_typ);

  std::string arm_name;
  priv_nh.getParam("arm_name", arm_name);

  std::string filename;
  priv_nh.getParam("filename", filename);


  // Robot control
  try {
    if (arm_type == ArmTypes::PSM1 || arm_type == ArmTypes::PSM2 || arm_type == ArmTypes::PSM3) {
      RobotServerPSM psm(nh, priv_nh, arm_type,
                         arm_name, RobotServerPSM::ACTIVE);
      psm.initRosCommunication();
      ros::spin();
      //psm.saveTrajectory(filename);
    }
    else {
      RobotServerDVRK arm(nh, priv_nh, arm_type,
                          arm_name, RobotServerDVRK::ACTIVE);
      arm.initRosCommunication();
      ros::spin();
      //arm.saveTrajectory(filename);
    }



    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}

