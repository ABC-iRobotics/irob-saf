#ifndef MIMIC_HPP
#define MIMIC_HPP

/*
 * 	mimic.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-07-19
 *
 *
 */


#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <irob_utils/tool_pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_general_robot/robot_client.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/SurgemeAction.h>


namespace saf {

class Mimic {

public:


protected:
  RobotClient primer_arm;
  RobotClient seconder_arm;
  ros::NodeHandle nh;

  static const double DEFAULT_LOOP_RATE;					// Hz


public:
  Mimic(ros::NodeHandle, std::string, std::string, double);		// dt
  ~Mimic();

  void mime();

};

}

#endif // MIMIC_HPP
