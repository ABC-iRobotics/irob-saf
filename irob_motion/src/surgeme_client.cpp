/*
 * 	surgeme_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-24
 *
 */

#include <irob_motion/surgeme_client.hpp>

namespace saf {


SurgemeClient::SurgemeClient(ros::NodeHandle nh, std::string arm_name):
  nh(nh), arm_name(arm_name),
  ac("surgeme/"+arm_name, true)
{

  // Subscribe and advertise topics

  subscribeTopics();
  advertiseTopics();
  waitForActionServer();
}

SurgemeClient::~SurgemeClient()
{
  // TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */

// Read pos 
void SurgemeClient::positionCartesianCurrentCB(
    const irob_msgs::ToolPoseStampedConstPtr& msg)
{
  position_cartesian_current = *msg;
  position_cartesian_current_pub.publish(msg);
}

// Read joints
void SurgemeClient::jointStateCurrentCB(
    const sensor_msgs::JointStateConstPtr& msg)
{
  joint_state_current = *msg;
  joint_state_current_pub.publish(msg);
}

void SurgemeClient::instrumentInfoCB(
    const irob_msgs::InstrumentInfoConstPtr& msg)
{
  instrument_info = *msg;
  instrument_info_pub.publish(msg);
}


void SurgemeClient::subscribeTopics()
{                 	            	
  position_cartesian_current_sub =
      nh.subscribe<irob_msgs::ToolPoseStamped>(
        "surgeme/"+arm_name+"/position_cartesian_current_cf",
        1000, &SurgemeClient::positionCartesianCurrentCB,this);

  joint_state_current_sub =
      nh.subscribe<sensor_msgs::JointState>(
        "surgeme/"+arm_name+"/joint_state_current",
        1000, &SurgemeClient::jointStateCurrentCB,this);

  instrument_info_sub =
      nh.subscribe<irob_msgs::InstrumentInfo>(
        "surgeme/"+arm_name+"/instrument_info",
        1000, &SurgemeClient::instrumentInfoCB,this);
}


void SurgemeClient::advertiseTopics()
{
  position_cartesian_current_pub
      = nh.advertise<irob_msgs::ToolPoseStamped>(
        "maneuver/"+arm_name+"/position_cartesian_current_cf",
        1000);

  joint_state_current_pub
      = nh.advertise<sensor_msgs::JointState>(
        "maneuver/"+arm_name+"/joint_state_current",
        1000);

  instrument_info_pub
      = nh.advertise<irob_msgs::InstrumentInfo>(
        "maneuver/"+arm_name+"/instrument_info",
        1000);
}


void SurgemeClient::waitForActionServer()
{
  ROS_INFO_STREAM("Wating for action server...");
  ac.waitForServer();
  ROS_INFO_STREAM("Action servers started");
}


ToolPose SurgemeClient::getPoseCurrent()
{
  while (position_cartesian_current.header.seq == 0)
  {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  ToolPose ret(position_cartesian_current);
  return ret;

}

sensor_msgs::JointState SurgemeClient::getJointStateCurrent()
{
  while (joint_state_current.header.seq == 0)
  {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  sensor_msgs::JointState ret(joint_state_current);
  return ret;

}


irob_msgs::InstrumentInfo SurgemeClient::getInstrumentInfo()
{
  while (instrument_info.name.empty())
  {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  irob_msgs::InstrumentInfo ret(instrument_info);
  return ret;
}

std::string SurgemeClient::getName()
{
  return arm_name;
}


/*
 *  irob_msgs/Surgeme action interfacing
 */

void SurgemeClient::stop()
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::STOP;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}

void SurgemeClient::nav_to_pos(ToolPose target,
                               double speed_cartesian,
                               std::vector<ToolPose> waypoints /* = empty */,
                               InterpolationMethod interp_method /* = LINEAR */)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::NAV_TO_POS;

  for (ToolPose p : waypoints)
    goal.waypoints.push_back(p.toRosPose());

  if (interp_method == InterpolationMethod::BEZIER)
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_BEZIER;
  else
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_LINEAR;

  goal.target = target.toRosPose();

  goal.speed_cartesian = speed_cartesian;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}

void SurgemeClient::grasp(ToolPose target, ToolPose approach_pose,
                          double target_diameter,
                          double compression_rate,
                          double speed_cartesian,
                          double speed_jaw,
                          std::vector<ToolPose> waypoints /* = empty */,
                          InterpolationMethod interp_method /* = LINEAR */)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::GRASP;

  goal.target = target.toRosPose();

  goal.approach_pose = approach_pose.toRosPose();

  for (ToolPose p : waypoints)
    goal.waypoints.push_back(p.toRosPose());

  if (interp_method == InterpolationMethod::BEZIER)
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_BEZIER;
  else
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_LINEAR;

  goal.target_diameter = target_diameter;
  goal.compression_rate = compression_rate;
  goal.speed_cartesian = speed_cartesian;
  goal.speed_jaw = speed_jaw;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}


void SurgemeClient::cut(ToolPose target, ToolPose approach_pose,
                        double target_diameter,
                        double speed_cartesian,
                        double speed_jaw,
                        std::vector<ToolPose> waypoints /* = empty */,
                        InterpolationMethod interp_method /* = LINEAR */)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::CUT;

  goal.target = target.toRosPose();
  goal.approach_pose = approach_pose.toRosPose();

  for (ToolPose p : waypoints)
    goal.waypoints.push_back(p.toRosPose());

  if (interp_method == InterpolationMethod::BEZIER)
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_BEZIER;
  else
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_LINEAR;

  goal.target_diameter = target_diameter;
  goal.speed_cartesian = speed_cartesian;
  goal.speed_jaw = speed_jaw;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}


void SurgemeClient::release(ToolPose approach_pose,	double target_diameter,
                            double speed_cartesian,
                            double speed_jaw)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::RELEASE;

  goal.approach_pose = approach_pose.toRosPose();


  goal.target_diameter = target_diameter;
  goal.speed_cartesian = speed_cartesian;
  goal.speed_jaw = speed_jaw;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}


void SurgemeClient::place(ToolPose target, ToolPose approach_pose,
                          double speed_cartesian,
                          std::vector<ToolPose> waypoints /* = empty */,
                          InterpolationMethod interp_method /* = LINEAR */)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::PLACE;

  goal.target = target.toRosPose();
  goal.approach_pose = approach_pose.toRosPose();

  for (ToolPose p : waypoints)
    goal.waypoints.push_back(p.toRosPose());

  if (interp_method == InterpolationMethod::BEZIER)
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_BEZIER;
  else
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_LINEAR;

  goal.speed_cartesian = speed_cartesian;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}


void SurgemeClient::push(ToolPose target, ToolPose approach_pose,
                         Eigen::Vector3d displacement,
                         double speed_cartesian,
                         double speed_jaw,
                         std::vector<ToolPose> waypoints /* = empty */,
                         InterpolationMethod interp_method /* = LINEAR */)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::PUSH;

  goal.target = target.toRosPose();
  goal.approach_pose = approach_pose.toRosPose();

  geometry_msgs::Point displacement_ros;
  displacement_ros.x = displacement.x();
  displacement_ros.y = displacement.y();
  displacement_ros.z = displacement.z();
  goal.displacement = displacement_ros;

  for (ToolPose p : waypoints)
    goal.waypoints.push_back(p.toRosPose());

  if (interp_method == InterpolationMethod::BEZIER)
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_BEZIER;
  else
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_LINEAR;

  goal.speed_cartesian = speed_cartesian;
  goal.speed_jaw = speed_jaw;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}


void SurgemeClient::dissect(ToolPose target, ToolPose approach_pose,
                            Eigen::Vector3d displacement,
                            double target_diameter,
                            double speed_cartesian,
                            double speed_jaw,
                            std::vector<ToolPose> waypoints /* = empty */,
                            InterpolationMethod interp_method /* = LINEAR */)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::DISSECT;

  goal.target = target.toRosPose();
  goal.approach_pose = approach_pose.toRosPose();

  geometry_msgs::Point displacement_ros;
  displacement_ros.x = displacement.x();
  displacement_ros.y = displacement.y();
  displacement_ros.z = displacement.z();
  goal.displacement = displacement_ros;

  for (ToolPose p : waypoints)
    goal.waypoints.push_back(p.toRosPose());

  if (interp_method == InterpolationMethod::BEZIER)
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_BEZIER;
  else
    goal.interpolation = irob_msgs::SurgemeGoal::INTERPOLATION_LINEAR;

  goal.target_diameter = target_diameter;
  goal.speed_cartesian = speed_cartesian;
  goal.speed_jaw = speed_jaw;

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}


void SurgemeClient::manipulate(Eigen::Vector3d displacement,
                               double speed_cartesian)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::MANIPULATE;

  geometry_msgs::Point displacement_ros;
  displacement_ros.x = displacement.x();
  displacement_ros.y = displacement.y();
  displacement_ros.z = displacement.z();
  goal.displacement = displacement_ros;

  goal.speed_cartesian = speed_cartesian;

  ac.sendGoal(goal);
  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}

void SurgemeClient::move_cam(Eigen::Vector3d marker_pos_tcp,
                             Eigen::Vector3d desired_pos_tcp,
                               double speed_cartesian)
{
  // Send a goal to the action
  irob_msgs::SurgemeGoal goal;

  goal.action = irob_msgs::SurgemeGoal::MOVE_CAM;
  goal.marker = wrapToMsg<geometry_msgs::Point>(marker_pos_tcp);
  goal.desired = wrapToMsg<geometry_msgs::Point>(desired_pos_tcp);
  goal.speed_cartesian = speed_cartesian;

  ac.sendGoal(goal);
  // Not waiting for action finish here, a notification will be received
  // in surgemeDoneCB
}

bool SurgemeClient::isSurgemeDone(bool spin /* = true */)
{
  return ac.isDone(spin);
}

actionlib::SimpleClientGoalState SurgemeClient::getState()
{
  return ac.getState();
}

irob_msgs::SurgemeFeedback SurgemeClient::getFeedback(bool spin /* = true */)
{
  return ac.getFeedback(spin);
}

irob_msgs::SurgemeResult SurgemeClient::getResult(bool spin /* = true */)
{
  return ac.getResult(spin);
}

}






















