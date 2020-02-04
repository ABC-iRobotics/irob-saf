/*
 * 	robot_server.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
 *
 *  Abstract base class for robot servers, interfacing irob_msgs/Robot actions to the actual robots.
 *
 *  Important note: initRosCommunication must be called
 *  in child class or main function!
 *
 */

#ifndef ROBOT_SERVER_HPP_
#define ROBOT_SERVER_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include <irob_utils/pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/topic_name_loader.hpp>
#include <actionlib/server/simple_action_server.h>
#include <irob_msgs/ToolPoseStamped.h>
#include <irob_msgs/RobotAction.h>
#include <irob_msgs/InstrumentInfo.h>
#include <irob_msgs/InstrumentJawPart.h>

namespace saf {

class RobotServer {

public:
  // Constants
  static const bool ACTIVE = true;
  static const bool PASSIVE = false;
  static constexpr const double INFO_PUB_RATE = 10.0;

protected:

  const std::string arm_name;
  ros::NodeHandle nh;
  bool isActive;

  // Action servers
  actionlib::SimpleActionServer<irob_msgs::RobotAction> as;

  // Hand-eye registration
  Eigen::Vector3d t;
  Eigen::Matrix3d R;

  // Surgical instrument information
  irob_msgs::InstrumentInfo instrument_info;

  // Publisher
  ros::Publisher position_cartesian_current_pub;
  ros::Publisher joint_state_current_pub;
  ros::Publisher instrument_info_pub;

  virtual void subscribeLowLevelTopics() = 0;
  virtual void advertiseLowLevelTopics() = 0;

  void advertiseHighLevelTopics()
  {
    // robot interface
    position_cartesian_current_pub
        = nh.advertise<irob_msgs::ToolPoseStamped>(
          "robot/"+arm_name+"/position_cartesian_current_cf",
          1000);
    joint_state_current_pub
        = nh.advertise<sensor_msgs::JointState>(
          "robot/"+arm_name+"/joint_state_current",
          1000);
    instrument_info_pub
        = nh.advertise<irob_msgs::InstrumentInfo>(
          "robot/"+arm_name+"/instrument_info",
          1000);
  }

  void startActionServer()
  {
    as.start();
  }

  void publishInfo()
  {
    ros::Rate loop_rate(INFO_PUB_RATE);
    while (ros::ok())
    {
      instrument_info_pub.publish(instrument_info);
      loop_rate.sleep();
    }
  }



public:

  void initRosCommunication()
  {
    subscribeLowLevelTopics();
    if (isActive == ACTIVE)
      advertiseLowLevelTopics();

    advertiseHighLevelTopics();
    startActionServer();
    boost::thread thread(boost::bind(&RobotServer::publishInfo, this));
  }

  void loadRegistration(ros::NodeHandle priv_nh)
  {
    std::vector<double> param_t;
    priv_nh.getParam("t", param_t);

    std::vector<double> param_R;
    priv_nh.getParam("R", param_R);

    for (int i = 0; i < t.rows(); i++)
      t(i) = param_t[i];

    for (int i = 0; i < R.rows(); i++)
      for (int j = 0; j < R.cols(); j++)
        R(i, j) = param_R[(i * R.rows()) + j];
    
    ROS_INFO_STREAM(
          "Registration read: "<< std::endl << t << std::endl << R);
  }

  void loadInstrumentInfo(ros::NodeHandle priv_nh)
  {

    priv_nh.getParam("instrument/name", instrument_info.name);
    priv_nh.getParam("instrument/jaw_length", instrument_info.jaw_length);

    std::string basic_type;
    priv_nh.getParam("instrument/basic_type", basic_type);

    if (basic_type == "GRIPPER")
      instrument_info.basic_type = irob_msgs::InstrumentInfo::GRIPPER;
    else if (basic_type == "SCISSORS")
      instrument_info.basic_type = irob_msgs::InstrumentInfo::SCISSORS;
    else if (basic_type == "CAMERA")
      instrument_info.basic_type = irob_msgs::InstrumentInfo::CAMERA;
    else
      throw std::runtime_error(
          "Invalid basic_type read from instrument info file.");

    int i = 0;
    double probe;
    while(priv_nh.getParam("instrument/jaw_parts/p"
                           + std::to_string(i)
                           + "/start", probe))
    {
      irob_msgs::InstrumentJawPart jaw_part;
      priv_nh.getParam("instrument/jaw_parts/p"
                       + std::to_string(i)
                       + "/start",
                       jaw_part.start);
      priv_nh.getParam("instrument/jaw_parts/p"
                       + std::to_string(i)
                       + "/end",
                       jaw_part.end);
      std::string type;
      priv_nh.getParam("instrument/jaw_parts/p"
                       + std::to_string(i)
                       + "/type",
                       type);
      if (type == "JOINT")
        jaw_part.type = irob_msgs::InstrumentJawPart::JOINT;
      else if (type == "GRIPPER")
        jaw_part.type = irob_msgs::InstrumentJawPart::GRIPPER;
      else if (type == "SCISSORS")
        jaw_part.type = irob_msgs::InstrumentJawPart::SCISSORS;
      else
        throw std::runtime_error(
            "Invalid instrument part type read from instrument info file.");

      instrument_info.jaw_parts.push_back(jaw_part);
      i++;
    }

    ROS_INFO_STREAM(
          "Instrument info read: "<< std::endl << instrument_info);

  }



  virtual void initArm() = 0;
  virtual void resetPose(bool) = 0;
  virtual void stop() = 0;
  virtual void followTrajectory(Trajectory<Pose>) = 0;
  virtual void moveJointAbsolute(sensor_msgs::JointState, double) = 0;
  virtual Pose getPoseCurrent() = 0;

  virtual void robotActionCB(const irob_msgs::RobotGoalConstPtr& goal)
  {
    switch(goal -> action)
    {
    case irob_msgs::RobotGoal::STOP:
    {
      stop();
      break;
    }

    case  irob_msgs::RobotGoal::INIT_ARM:
    {
      initArm();
      break;
    }

    case  irob_msgs::RobotGoal::RESET_POSE:
    {
      resetPose(goal -> move_allowed);
      break;
    }

    case  irob_msgs::RobotGoal::FOLLOW_TRAJECTORY:
    {
      followTrajectory(goal->trajectory);
      break;
    }
    case  irob_msgs::RobotGoal::MOVE_JOINT:
    {
      moveJointAbsolute(goal->joint_state, 0.01);
      break;
    }
    default:
    {
      ROS_ERROR_STREAM(arm_name  <<
                       ": invalid robot action code received");
      irob_msgs::RobotResult result;
      result.pose = getPoseCurrent().toRosToolPose();
      result.info = "invalid robot action code";
      as.setAborted(result);
      break;
    }
    }
  }

  // initRosCommunication must be called in child class or main function
  RobotServer(ros::NodeHandle nh, ros::NodeHandle priv_nh,
              std::string arm_name, bool isActive):
    nh(nh), arm_name(arm_name), isActive(isActive),
    as(nh, "robot/"+arm_name+"/robot_action",
       boost::bind(&RobotServer::robotActionCB, this, _1), false)
  {
    loadRegistration(priv_nh);
    loadInstrumentInfo(priv_nh);
  }

  ~RobotServer() {}


};

}
#endif /* ROBOT_SERVER_HPP_ */
