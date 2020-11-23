/*
 * 	robot_client.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-19
 *
 *	Single arm, ROS actions interface to the robot server. This objects are
 *  planned to be used as members of irob_motion/SurgemeServer.
 *	Substribes to cartesian_pos.
 *	Move relative, absolute, gripper, waypoints, Bezier...
 */

#ifndef ROBOT_CLIENT_HPP_
#define ROBOT_CLIENT_HPP_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <irob_utils/pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/trajectory_factory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/irob_action_client.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/RobotAction.h>
#include <irob_msgs/InstrumentInfo.h>
#include <irob_msgs/InstrumentJawPart.h>
#include <sensor_msgs/JointState.h>

namespace saf {

class RobotClient {

public:
  typedef enum CoordFrame
  {WCS, TCPF} CoordFrame;


private:
  const std::string arm_name;
  ros::NodeHandle nh;

  double dt;

  // Action clients
  IrobActionClient<irob_msgs::RobotAction> ac;


  // States
  irob_msgs::ToolPoseStamped position_cartesian_current;
  sensor_msgs::JointState joint_state_current;
  irob_msgs::InstrumentInfo instrument_info;

  // Subscribers
  ros::Subscriber position_cartesian_current_sub;
  ros::Subscriber joint_state_current_sub;
  ros::Subscriber instrument_info_sub;

  // Publishers
  ros::Publisher position_cartesian_current_pub;
  ros::Publisher joint_state_current_pub;
  ros::Publisher instrument_info_pub;

  void subscribeTopics();
  void advertiseTopics();
  void waitForActionServer();


public:
  RobotClient(ros::NodeHandle, std::string, double);
  ~RobotClient();

  // Callbacks

  void positionCartesianCurrentCB(
      const irob_msgs::ToolPoseStampedConstPtr&);

  void jointStateCurrentCB(
      const sensor_msgs::JointStateConstPtr&);

  void instrumentInfoCB(
      const irob_msgs::InstrumentInfoConstPtr&);

  Pose getPoseCurrent();
  sensor_msgs::JointState getJointStateCurrent();
  irob_msgs::InstrumentInfo getInstrumentInfo();
  std::string getName();

  // Robot motions
  void resetPose(bool);
  void moveJaws(double, double);
  void moveTool(Pose, double, std::vector<Pose> = std::vector<Pose>(),
                InterpolationMethod = LINEAR);
  void moveJoints(sensor_msgs::JointState);

  void stop();


  bool isActionDone(bool = true);
  actionlib::SimpleClientGoalState getState();

  irob_msgs::RobotFeedback getFeedback(bool = true);
  irob_msgs::RobotResult getResult(bool = true);

};

}
#endif /* ROBOT_CLIENT_HPP_ */
