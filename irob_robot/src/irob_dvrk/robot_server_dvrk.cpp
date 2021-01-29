/*
 *  robot_server_dvrk.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
 *
 */

#include <irob_dvrk/robot_server_dvrk.hpp>
#include <numeric>
#include <chrono>

namespace saf {


const std::string RobotServerDVRK::READY
= "READY";



RobotServerDVRK::RobotServerDVRK(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                                 ArmTypes arm_typ,
                                 std::string arm_name,
                                 bool isActive):
  RobotServer(nh, priv_nh, arm_name, isActive), arm_typ(arm_typ){}

RobotServerDVRK::~RobotServerDVRK() {}


/*
 * Move in joint space immediately.
 */

void RobotServerDVRK::resetPose(bool move_allowed)
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
  feedback.pose = getPoseCurrent().toRosToolPose();
  as.publishFeedback(feedback);

  if(success)
  {
    result.info = "done";
    result.pose = getPoseCurrent().toRosToolPose();
    ROS_INFO_STREAM(arm_typ.name << " pose reset succeeded");
    // set the action state to succeeded
    as.setSucceeded(result);
  }
}

void RobotServerDVRK::stop()
{
  // helper variables
  bool success = false;

  irob_msgs::RobotFeedback feedback;
  irob_msgs::RobotResult result;

  ROS_INFO_STREAM("Stopping " << arm_typ.name);
  
  // Do not check that preempt has not been requested by the client

  success = true;
  // Send some feedback
  feedback.info = "stopped";
  feedback.pose = getPoseCurrent().toRosToolPose();
  as.publishFeedback(feedback);

  if(success)
  {
    result.info = "stopped";
    result.pose = getPoseCurrent().toRosToolPose();
    ROS_INFO_STREAM(arm_typ.name << " stopped");
    // set the action state to succeeded
    as.setSucceeded(result);
  }
}



void RobotServerDVRK::followTrajectory(Trajectory<ToolPose> tr)
{
  // helper variables
  bool success = true;
  irob_msgs::RobotFeedback feedback;
  irob_msgs::RobotResult result;

  ROS_INFO_STREAM("Starting trajectory follow action.");
  ROS_INFO_STREAM("Goal untransformed:" << tr[tr.size() - 1]);
  // Go to m-s from mm-s here
  // Hande-eye calibration
  tr.transform(R, t, 0.001);
  ROS_INFO_STREAM("Goal transformed:" << tr[tr.size() - 1]);

  ros::Rate loop_rate(1.0/tr.dt);
  // start executing the action
  for (int i = 0; i < tr.size(); i++)
  {
    // check that preempt has not been requested by the client
    if (as.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO_STREAM("Follow trajectory: Preempted");
      // set the action state to preempted
      as.setPreempted();
      success = false;
      break;
    }
    try {
      moveCartesianAbsolute(tr[i],tr.dt);
      feedback.info = "following trajectory, index: " + i;
      feedback.pose = tr[i].toRosToolPose();
      as.publishFeedback(feedback);
    }catch (std::runtime_error e)
    {
      result.pose = getPoseCurrent().toRosToolPose();
      result.info = e.what();
      ROS_ERROR_STREAM(arm_typ.name << ": an error occured: " << e.what());
      // set the action state to succeeded
      as.setAborted(result);
      success = false;
      break;
    }

    loop_rate.sleep();
  }

  if(success)
  {
    result.pose = getPoseCurrent().toRosToolPose();
    result.info = "done";
    ROS_INFO_STREAM("Follow trajectory: Succeeded");
    // set the action state to succeeded
    as.setSucceeded(result);
  }
}


/*
 * Callbacks
 */

void RobotServerDVRK::robotStateCB(const std_msgs::String msg)
{
  current_state = msg;
}

void RobotServerDVRK::stateJointCurrentCB(const sensor_msgs::JointStateConstPtr& msg) 
{
  position_joint = *msg;
  sensor_msgs::JointState fwd;
  fwd = position_joint;
  //ROS_INFO_STREAM(*msg);
  //ROS_INFO_STREAM(fwd);
  joint_state_current_pub.publish(fwd);
}

void RobotServerDVRK::positionCartesianCurrentCB(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
  position_cartesian_current = *msg;
  irob_msgs::ToolPoseStamped fwd;
  fwd.header = position_cartesian_current.header;

  ToolPose tmp(position_cartesian_current, 0.0);

  // Hand-eye calibration
  // Convert from m-s to mm-s
  fwd.pose = tmp.invTransform(R, t, 0.001).toRosToolPose();
  position_cartesian_current_pub.publish(fwd);
}

void RobotServerDVRK::errorCB(const std_msgs::String msg) 
{
  error = msg;
}

void RobotServerDVRK::warningCB(const std_msgs::String msg) 
{
  warning = msg;
}

void RobotServerDVRK::subscribeLowLevelTopics() 
{
  current_state_sub = nh.subscribe<std_msgs::String>(
        TopicNameLoader::load(nh,
                              arm_typ.name,
                              "dvrk_topics/current_state"),
        1000, &RobotServerDVRK::robotStateCB,this);
  
  state_joint_current_sub = nh.subscribe<sensor_msgs::JointState>(
        TopicNameLoader::load(nh,
                              arm_typ.name,
                              "dvrk_topics/state_joint_current"),
        1000, &RobotServerDVRK::stateJointCurrentCB,this);

  position_cartesian_current_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        TopicNameLoader::load(nh,
                              arm_typ.name,
                              "dvrk_topics/position_cartesian_current"),
        1000, &RobotServerDVRK::positionCartesianCurrentCB,this);

  error_sub = nh.subscribe<std_msgs::String>(
        TopicNameLoader::load(nh,
                              arm_typ.name,
                              "dvrk_topics/error"),
        1000, &RobotServerDVRK::errorCB,this);

  warning_sub = nh.subscribe<std_msgs::String>(
        TopicNameLoader::load(nh,
                              arm_typ.name,
                              "dvrk_topics/warning"),
        1000, &RobotServerDVRK::warningCB,this);

}

void RobotServerDVRK::advertiseLowLevelTopics() 
{
  // dVRK
  position_joint_pub = nh.advertise<sensor_msgs::JointState>(
        TopicNameLoader::load(nh,
                              arm_typ.name,
                              "dvrk_topics/set_position_joint"),
        1000);
  position_cartesian_pub = nh.advertise<geometry_msgs::TransformStamped>(
        TopicNameLoader::load(nh,
                              arm_typ.name,
                              "dvrk_topics/set_position_cartesian"),
        1000);
}


/*
 * DVRK actions
 */

std::string RobotServerDVRK::getCurrentState()
{
  return current_state.data;
}

double RobotServerDVRK::getJointStateCurrent(int index)
{
  ros::spinOnce();
  if (index > ((int)position_joint.position.size())-1)
  {
    ROS_WARN_STREAM("Joint index too big or no joint position information.");
    return NAN;
  }
  return position_joint.position[index];
}

std::vector<double> RobotServerDVRK::getJointStateCurrent()
{
  ros::spinOnce();
  std::vector<double> ret(arm_typ.dof);

  ret = position_joint.position;
  return ret;
}

Eigen::Vector3d RobotServerDVRK::getPositionCartesianCurrent()
{
  ros::spinOnce();
  Eigen::Vector3d ret(position_cartesian_current.pose.position.x,
                      position_cartesian_current.pose.position.y,
                      position_cartesian_current.pose.position.z);
  return ret;
}

Eigen::Quaternion<double> RobotServerDVRK::getOrientationCartesianCurrent()
{
  ros::spinOnce();
  Eigen::Quaternion<double> ret(position_cartesian_current.pose.orientation.x,
                                position_cartesian_current.pose.orientation.y,
                                position_cartesian_current.pose.orientation.z,
                                position_cartesian_current.pose.orientation.w);
  return ret;
}

ToolPose RobotServerDVRK::getPoseCurrent()
{
  ros::spinOnce();
  ToolPose ret(position_cartesian_current, 0.0);
  return ret;

}


/*
 * Move in joint space immediately.
 */
void RobotServerDVRK::moveJointAbsolute(sensor_msgs::JointState pos, double dt)
{
  // Collect data
  if (!pos.position.empty()){
  std::vector<double> currJoint = getJointStateCurrent();
  sensor_msgs::JointState new_position_joint = pos;
  //for (int i = 0; i <  arm_typ.dof; i++)
    //new_position_joint.position[i] = pos.position[i];

  //new_position_joint = maximizeVelJoint(new_position_joint, currJoint, dt);
  // Safety
  try {

    checkErrors();
    checkNaNJoint(new_position_joint);
    // End safety

    // Publish movement

    position_joint_pub.publish(new_position_joint);
    //ROS_INFO_STREAM(new_position_joint);
    ros::spinOnce();
    irob_msgs::RobotResult result;
    result.pose = getPoseCurrent().toRosToolPose();
    result.info = "done";
    //ROS_INFO_STREAM("Follow trajectory: Succeeded");
      // set the action state to succeeded
   as.setSucceeded(result);

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

/*
 * Move in cartesian immediately.
 */
void RobotServerDVRK::moveCartesianRelative(Eigen::Vector3d movement, double dt)
{
  // Collect data
  ToolPose currPose = getPoseCurrent();
  ToolPose pose = currPose;
  pose += movement;
  geometry_msgs::Pose new_position_cartesian = pose.toRosPose();

  // Safety
  try {
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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
  //ros::Duration(0.1).sleep();

}

/*
 * Move in cartesian immediately.
 */
void RobotServerDVRK::moveCartesianAbsolute(Eigen::Vector3d position, double dt)
{
  // Collect data
  ToolPose currPose = getPoseCurrent();
  ToolPose pose = currPose;
  pose.position = position;
  geometry_msgs::Pose new_position_cartesian = pose.toRosPose();

  // Safety
  try {
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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
  //ros::Duration(0.1).sleep();

}

/*
 * Move in cartesian immediately.
 */
void RobotServerDVRK::moveCartesianAbsolute(Eigen::Quaternion<double> orientation, double dt)
{
  // Collect data
  ToolPose currPose = getPoseCurrent();
  ToolPose pose = currPose;
  pose.orientation = orientation;
  geometry_msgs::Pose new_position_cartesian = pose.toRosPose();

  // Safety
  try {
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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
void RobotServerDVRK::moveCartesianAbsolute(ToolPose pose, double dt)
{
  // Collect data
  ToolPose currPose = getPoseCurrent();
  geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
  try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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

void RobotServerDVRK::checkErrors()
{
  if (!warning.data.empty())
  {
    ROS_WARN_STREAM(warning.data);
    warning.data.clear();
  }

  if (!error.data.empty())
  {
    std::string err_data = error.data;
    error.data.clear();
    throw std::runtime_error(err_data);
  }
}

void RobotServerDVRK::checkVelCartesian(const ToolPose& pose,
                                        const ToolPose& currPose, double dt)
{
  ToolPose::Distance d = currPose.dist(pose)/dt;
  if (d.cartesian > arm_typ.maxVelPose.cartesian ||
      d.angle > arm_typ.maxVelPose.angle ||
      d.jaw > arm_typ.maxVelPose.jaw)
  {
    std::stringstream errstring;
    errstring << "Desired pose too far from current."<< std::endl
              << "Desired pose:\t" << pose << std::endl
              << "Current pose:\t" << currPose << std::endl
              << "Velocity:\t" << d << std::endl
              << "MaxVelocity:\t" << arm_typ.maxVelPose << std::endl;
    //ROS_ERROR_STREAM(errstring.str());
    throw std::runtime_error(errstring.str());
  }
}

void RobotServerDVRK::checkNaNCartesian(const ToolPose& pose)
{
  if (pose.isNaN())
  {
    std::stringstream errstring;
    errstring << "Desired pose is NaN:\t"<< std::endl
              << pose << std::endl;
    //ROS_ERROR_STREAM(errstring.str());
    throw std::runtime_error(errstring.str());
  }
}

void RobotServerDVRK::checkVelJoint(const sensor_msgs::JointState& new_position_joint, 
                                    const std::vector<double>& currJoint, double dt)
{
  std::vector<double> distance(arm_typ.dof);
  for (int i = 0; i < arm_typ.dof; i++) {
    distance[i] = std::abs(new_position_joint.position[i]-currJoint[i])/dt;
  }
  for (int i = 0; i < arm_typ.dof; i++)
  {
    if (distance[i] > arm_typ.maxVelJoint[i])
    {
      std::stringstream errstring;
      errstring << "Desired joint vector too far from current."
                << std::endl
                << "Desired joint vector:\t"
                << new_position_joint.position
                << std::endl
                << "Current joint vector:\t"
                << currJoint
                << std::endl
                << "Velocity:\t"
                << distance
                << std::endl
                << "MaxVelocity:\t"
                << arm_typ.maxVelJoint
                << std::endl;
      //ROS_ERROR_STREAM(errstring.str());
      throw std::runtime_error(errstring.str());
    }
  }
}

sensor_msgs::JointState RobotServerDVRK::maximizeVelJoint(const sensor_msgs::JointState& new_position_joint,
                                    const std::vector<double>& currJoint, double dt)
{
  std::vector<double> distance(arm_typ.dof);
  sensor_msgs::JointState ret(new_position_joint);

  for (int i = 0; i < arm_typ.dof; i++) {
    distance[i] = new_position_joint.position[i]-currJoint[i]/dt;
  }
  for (int i = 0; i < arm_typ.dof; i++)
  {
    if (std::abs(distance[i]) > arm_typ.maxVelJoint[i])
    {
      ret.position[i] = currJoint[i] + std::copysign(arm_typ.maxVelJoint[i], distance[i]);
    }
  }
  return ret;
}

void RobotServerDVRK::checkNaNJoint(const sensor_msgs::JointState& new_position_joint)
{
  bool foundNaN = false;
  for (int i = 0; i < arm_typ.dof; i++) {
    foundNaN = foundNaN || std::isnan(new_position_joint.position[i]);
  }
  if (foundNaN)
  {
    std::stringstream errstring;
    errstring << "Desired joint vector is NaN:\t"
              << new_position_joint.position
              << std::endl;
    //ROS_ERROR_STREAM(errstring.str());
    throw std::runtime_error(errstring.str());
  }
}


void RobotServerDVRK::recordTrajectory(Trajectory<Eigen::Vector3d>& tr) 
{
  ros::Rate loop_rate(1.0/tr.dt);
  // Skip invalid points
  while (ros::ok() && getPositionCartesianCurrent().norm() < 0.001)
  {
    loop_rate.sleep();
  }
  while(ros::ok())
  {
    tr.addPoint(getPositionCartesianCurrent());
    loop_rate.sleep();
  }
}

void RobotServerDVRK::recordTrajectory(Trajectory<ToolPose>& tr)
{
  ros::Rate loop_rate(1.0/tr.dt);
  // Skip invalid points
  while (ros::ok() && getPositionCartesianCurrent().norm() < 0.001)
  {
    loop_rate.sleep();
  }
  while(ros::ok())
  {
    tr.addPoint(getPoseCurrent());
    loop_rate.sleep();
  }


}

void RobotServerDVRK::saveTrajectory(std::string filename)
{
  std::ofstream logfile;
  logfile.open (filename, std::ofstream::out | std::ofstream::trunc);
  if (logfile.is_open())
    ROS_INFO_STREAM("Start logging to "<< filename);
  else
    ROS_INFO_STREAM("Cannot open file  "<< filename);

  ros::Rate loop_rate(100.0);
  // Skip invalid points
  while (ros::ok() && getPositionCartesianCurrent().norm() < 0.001)
  {
    loop_rate.sleep();
  }
  while(ros::ok())
  {
    logfile << getPoseCurrent() << std::endl;
    loop_rate.sleep();
  }
  logfile.flush();
  logfile.close();
  ROS_INFO_STREAM("Trajectory saved to file  "<< filename << " successfully.");
}

}

