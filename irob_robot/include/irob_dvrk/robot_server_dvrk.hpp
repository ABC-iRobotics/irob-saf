/*
 * 	robot_server_dvrk.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
 *
 *  Base class for dVRK robot arms, itself usable for
 *  MTMs and ECM.
 *
 */

#ifndef ROBOT_SERVER_DVRK_HPP_
#define ROBOT_SERVER_DVRK_HPP_

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include <irob_dvrk/arm_types.hpp>
#include <irob_general_robot/robot_server.hpp>

namespace saf {

class RobotServerDVRK: public RobotServer {

public:

  // Constants
  static const std::string READY;


protected:
  const ArmTypes arm_typ;


  // States
  std_msgs::String status;
  sensor_msgs::JointState measured_js;
  geometry_msgs::PoseStamped measured_cp;
  std_msgs::String error;
  std_msgs::String warning;

  // Subscribers
  ros::Subscriber status_sub;
  ros::Subscriber smeasured_js_sub;
  ros::Subscriber measured_cp_sub;
  ros::Subscriber error_sub;
  ros::Subscriber warning_sub;


  // Publishers
  ros::Publisher position_joint_pub;
  ros::Publisher position_cartesian_pub;

  void subscribeLowLevelTopics();
  void advertiseLowLevelTopics();

public:
  RobotServerDVRK(ros::NodeHandle, ros::NodeHandle, ArmTypes, std::string, bool);
  ~RobotServerDVRK();

  // Callbacks
  void resetPose(bool);
  void stop();
  void followTrajectory(Trajectory<ToolPose>);
  void moveJointAbsolute(sensor_msgs::JointState , double );

  void status_cb(const std_msgs::String);
  void error_cb(const std_msgs::String);
  void warning_cb(const std_msgs::String);
  void measured_js_cb(const sensor_msgs::JointStateConstPtr&);
  virtual void measured_cp_cb(
      const geometry_msgs::PoseStampedConstPtr&);


  void loadRegistration(std::string);

  double getJointStateCurrent(int);
  std::vector<double> getJointStateCurrent();
  Eigen::Vector3d getPositionCartesianCurrent();
  Eigen::Quaternion<double> getOrientationCartesianCurrent();
  ToolPose getPoseCurrent();

  //DVRK actions
  std::string getCurrentState();
  void moveCartesianRelative(Eigen::Translation3d, double = 0.01);
  virtual void moveCartesianAbsolute(ToolPose, double = 0.01);


  void recordTrajectory(Trajectory<Eigen::Vector3d>&);
  void recordTrajectory(Trajectory<ToolPose>&);
  void saveTrajectory(std::string);


  void checkErrors();
  void checkVelCartesian(const ToolPose&, const ToolPose&, double);
  void checkNaNCartesian(const ToolPose&);
  void checkVelJoint(const sensor_msgs::JointState&,
                     const std::vector<double>&, double);
  sensor_msgs::JointState maximizeVelJoint(const sensor_msgs::JointState&,
                     const std::vector<double>&, double);
  void checkNaNJoint(const sensor_msgs::JointState&);

};

}
#endif /* ROBOT_SERVER_DVRK_HPP_ */
