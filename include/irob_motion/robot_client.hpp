/*
 * 	robot_client.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-19
 *
 *	Single arm, ROS actions down to the robot interface. Property of
 * 	GestureServer.
 *	Substribe to cartesian_pos.
 *	Move relative, absolute, gripper, waypoints, Bezier...
 */

#ifndef ROBOT_CLIENT_HPP_
#define ROBOT_CLIENT_HPP_

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
#include "irob_utils/pose.hpp"
#include "irob_utils/trajectory.hpp"
#include "irob_utils/utils.hpp"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_autosurg/ToolPoseStamped.h>

#include <irob_autosurg/InitArmAction.h>
#include <irob_autosurg/ResetPoseAction.h>
#include <irob_autosurg/FollowTrajectoryAction.h>

using namespace ias;

class RobotClient {

public:
	typedef enum CoordFrame 
    	{WCS, TCPF} CoordFrame;  
   

private:
	const std::string arm_name;
    ros::NodeHandle nh;
    
    double dt;
    
    // Action clients
    actionlib::SimpleActionClient<irob_autosurg::InitArmAction> 
    									init_arm_ac("init_arm", true);
    actionlib::SimpleActionClient<irob_autosurg::InitArmAction>
    									reset_pose_ac("reset_pose", true);
   	actionlib::SimpleActionClient<irob_autosurg::FollowTrajectoryAction> 
   										follow_tr_ac("follow_trajectory", true);
   

    // States
    irob_autosurg::ToolPoseStamped position_cartesian_current;

    // Subscribers
    ros::Subscriber position_cartesian_current_sub;



    // Publishers
	ros::Publisher position_cartesian_current_pub;
   	
    void subscribeTopics();
    void advertiseTopics();
    void waitForActionServers();
    

public:
	RobotClient(ros::NodeHandle, std::string, double);
	~RobotClient();

    // Callbacks    

    void positionCartesianCurrentCB(const geometry_msgs::PoseStampedConstPtr&);

   	Pose getPoseCurrent();
   	
   	// Robot motions
   	void initArm(bool, bool);	
   	void resetPose(bool);
	void moveGripper(double, double = 10.0);
	void goTo(Pose, double = 10.0, vector<Pose> = vector<Pose>(), 
			InterpolatonMethod = InterpolationMethod.LINEAR);
	void moveRelative(Pose, CoordFrame);
	void moveRelative(Eigen::Vector3d, CoordFrame);
	void moveRelative(Eigen::Quaternion<double>, CoordFrame);
	
};


#endif /* ROBOT_CLIENT_HPP_ */
