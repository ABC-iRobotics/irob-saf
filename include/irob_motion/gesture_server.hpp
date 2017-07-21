/*
 * 	gesture_server.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-18
 *	
 *	Separated ROS node to support preempted actions.
 *	close gripper, penetrate, goto
 */

#ifndef GESTURE_SERVER_HPP_
#define GESTURE_SERVER_HPP_

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
#include "irob_motion/robot_client.hpp"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <irob_autosurg/ToolPoseStamped.h>

#include <irob_autosurg/CloseToolAction.h>
#include <irob_autosurg/OpenToolAction.h>
#include <irob_autosurg/PenetrateAction.h>
#include <irob_autosurg/GoToAction.h>

using namespace ias;

class GestureServer {

public:

   

protected:
	RobotClient arm;
    ros::NodeHandle nh;
    
    // Action servers
    actionlib::SimpleActionServer<irob_autosurg::CloseToolAction>
    	 close_tool_as;
   	actionlib::SimpleActionServer<irob_autosurg::OpenToolAction>
    	 open_tool_as;
   	actionlib::SimpleActionServer<irob_autosurg::PenetrateAction>
    	 penetrate_as;
    actionlib::SimpleActionServer<irob_autosurg::GoToAction>
    	 go_to_as;
   	

    void startActionServers();

public:
	GestureServer(ros::NodeHandle, std::string, double);		// dt
	~GestureServer();

    // Callbacks

    void closeToolActionCB(
    		const irob_autosurg::GraspGoalConstPtr &);
    		
   	void openToolActionCB(
    		const irob_autosurg::ReleseGoalConstPtr &);
    		
   	void penetrateActionCB(
    		const irob_autosurg::PenetrateGoalConstPtr &);
    		
   	void goToActionCB(
    		const irob_autosurg::GoToGoalConstPtr &);
    

   	Pose getPoseCurrent();
   	std::string getArmName();
	
};


#endif /* GESTURE_SERVER_HPP_ */
