/*
 * 	robot_server_psm.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
 *
 *  Robot server for dVRK PSMs.
 *
 */

#ifndef ROBOT_SERVER_PSM_HPP_
#define ROBOT_SERVER_PSM_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Float32.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include <irob_utils/utils.hpp>
#include <irob_dvrk/robot_server_dvrk.hpp>
#include <irob_dvrk/arm_types.hpp>
#include <irob_utils/topic_name_loader.hpp>
#include <irob_utils/tool_pose.hpp>
#include <irob_utils/trajectory.hpp>

namespace saf {

class RobotServerPSM: public RobotServerDVRK {


private:           

  // Publishers
  ros::Publisher position_jaw_pub;

  // Subscribers
  ros::Publisher position_jaw_sub;

  // States
  sensor_msgs::JointState jaw_measured_js;

  void advertiseLowLevelTopics();
  void subscribeLowLevelTopics();

public:
  RobotServerPSM(ros::NodeHandle, ros::NodeHandle, ArmTypes, std::string, bool);
  ~RobotServerPSM();

  void resetPose(bool);

  void measured_cp_cb(
      const geometry_msgs::TransformStampedConstPtr&) ;

  void positionJawCurrentCB(
      const sensor_msgs::JointStateConstPtr&) ;


  ToolPose getPoseCurrent();

  void moveCartesianAbsolute(ToolPose, double = 0.01);
  void moveJawRelative(double, double = 0.01);
  void moveJawAbsolute(double, double = 0.01);

};

}
#endif /* ROBOT_SERVER_PSM_HPP_ */
