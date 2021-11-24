/*
 * 	surgeme_server.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-06
 *
 *	Separated ROS node to support preempted irob_msgs/Surgeme actions.
 *  Contains the implementation of surgemes, like
 *	grasp, release, cut, nav_to_pos, etc.
 *  Handles instrument info to decide is the surgeme is executable with the
 *  installed instrument, and calculate using the length of jaws.
 *
 */

#ifndef SURGAME_SERVER_HPP_
#define SURGAME_SERVER_HPP_

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
#include <irob_utils/abstract_directions.hpp>
#include <irob_utils/utils.hpp>
#include <irob_general_robot/robot_client.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <irob_msgs/ToolPoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <irob_msgs/SurgemeAction.h>


namespace saf {

class SurgemeServer {

public:

  struct SurgemeSetting {
    double jaw_open_angle;
    double jaw_closed_angle;
    Eigen::Vector3d t;	// translation

    friend std::ostream& operator<<(std::ostream&, const SurgemeSetting&);
  };


protected:
  RobotClient arm;
  ros::NodeHandle nh;
  tf2_ros::TransformBroadcaster br_ori;
  tf2_ros::TransformBroadcaster br_new;

  // Action servers
  actionlib::SimpleActionServer<irob_msgs::SurgemeAction> as;

  static const double DEFAULT_LOOP_RATE;					// Hz


public:
  SurgemeServer(ros::NodeHandle, std::string, double);		// dt
  ~SurgemeServer();

  // Callbacks

  void surgemeActionCB(
      const irob_msgs::SurgemeGoalConstPtr &);

  ToolPose getPoseCurrent();

  sensor_msgs::JointState getJointStateCurrent();

  std::string getArmName();

protected:

  // Methods for surgeme execution
  void stop();

  void nav_to_pos(ToolPose ,std::vector<ToolPose>, InterpolationMethod, double);

  void grasp(ToolPose, ToolPose, double, double, std::vector<ToolPose>,
             InterpolationMethod,
             double, double);

  void cut(ToolPose, ToolPose, double, std::vector<ToolPose>,
           InterpolationMethod,
           double, double);

  void push(ToolPose, ToolPose, Eigen::Vector3d, std::vector<ToolPose>,
            InterpolationMethod,
            double, double);

  void dissect(ToolPose, ToolPose, Eigen::Vector3d,double, std::vector<ToolPose>,
               InterpolationMethod,
               double, double);

  void release(ToolPose, double,
               double, double);

  void place(ToolPose, ToolPose, double, std::vector<ToolPose>, InterpolationMethod,
             double);

  void manipulate(Eigen::Vector3d,
                  double);

  void move_cam(Eigen::Vector3d, Eigen::Vector3d,
                  double);

  bool waitForActionDone(std::string);
  void handleActionState(std::string, bool = false);
  bool isAbleToDoSurgeme(int);
  irob_msgs::InstrumentJawPart findInstrumentJawPartForSurgeme(int);
  SurgemeSetting calcSurgemeSetting(int, irob_msgs::InstrumentJawPart,
                                    Eigen::Quaternion<double>, double, double = 1.0);

};

}
#endif /* SURGAME_SERVER_HPP_ */
