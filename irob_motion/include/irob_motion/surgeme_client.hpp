/*
 * 	gesture_client.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-20
 *
 *	Single arm, ROS actions interface to the surgeme server. This objects are
 *  planned to be used as members of subtask-level logic.
 *
 */

#ifndef SURGEME_CLIENT_HPP_
#define SURGEME_CLIENT_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <irob_utils/tool_pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/irob_action_client.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/SurgemeAction.h>
#include <irob_msgs/InstrumentInfo.h>
#include <irob_msgs/InstrumentJawPart.h>
#include <sensor_msgs/JointState.h>

namespace saf {

class SurgemeClient {

public:

protected:
  const std::string arm_name;
  ros::NodeHandle nh;

  // Action clients
  IrobActionClient<irob_msgs::SurgemeAction> ac;


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
  void startActionClients();
  void waitForActionServer();

public:
  SurgemeClient(ros::NodeHandle, std::string);
  ~SurgemeClient();

  // Callbacks

  void positionCartesianCurrentCB(
      const irob_msgs::ToolPoseStampedConstPtr&);

  void jointStateCurrentCB(
      const sensor_msgs::JointStateConstPtr&);

  void instrumentInfoCB(
      const irob_msgs::InstrumentInfoConstPtr&);

  ToolPose getPoseCurrent();
  sensor_msgs::JointState getJointStateCurrent();
  irob_msgs::InstrumentInfo getInstrumentInfo();
  std::string getName();

  // Robot motions
  void stop();
  void nav_to_pos(Eigen::Affine3d,
                  double,
                  std::vector<Eigen::Affine3d> = std::vector<Eigen::Affine3d>(),
                  InterpolationMethod = InterpolationMethod::LINEAR);
  void grasp(Eigen::Affine3d, Eigen::Affine3d, double,	double,
             double,
             double,
             std::vector<Eigen::Affine3d> = std::vector<Eigen::Affine3d>(),
             InterpolationMethod = InterpolationMethod::LINEAR);
  void cut(Eigen::Affine3d, Eigen::Affine3d,double,
           double,
           double,
           std::vector<Eigen::Affine3d> = std::vector<Eigen::Affine3d>(),
           InterpolationMethod = InterpolationMethod::LINEAR);
  void release(Eigen::Affine3d,	double,
               double,
               double);
  void place(Eigen::Affine3d, Eigen::Affine3d,
             double,
             std::vector<Eigen::Affine3d> = std::vector<Eigen::Affine3d>(),
             InterpolationMethod = InterpolationMethod::LINEAR);
  void push(Eigen::Affine3d, Eigen::Affine3d,
            Eigen::Vector3d,
            double,
            double,
            std::vector<Eigen::Affine3d> = std::vector<Eigen::Affine3d>(),
            InterpolationMethod = InterpolationMethod::LINEAR);
  void dissect(Eigen::Affine3d, Eigen::Affine3d,
               Eigen::Vector3d,
               double,
               double,
               double,
               std::vector<Eigen::Affine3d> = std::vector<Eigen::Affine3d>(),
               InterpolationMethod = InterpolationMethod::LINEAR);

  void manipulate(Eigen::Vector3d,
                  double);

  void move_cam(Eigen::Vector3d, Eigen::Vector3d,
                  double);

  bool isSurgemeDone(bool = true);
  actionlib::SimpleClientGoalState getState();

  irob_msgs::SurgemeFeedback getFeedback(bool = true);
  irob_msgs::SurgemeResult getResult(bool = true);


};

}
#endif /* SURGEME_SERVER_HPP_ */
