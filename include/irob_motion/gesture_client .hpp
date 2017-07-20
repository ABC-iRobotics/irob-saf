/*
 * 	gesture_client.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-20
 *
 *	Property of ManeuverServer.
 *
 *	TODO Handle tool types, allow gestures etc?
 */

#ifndef GESTURE_CLIENT_HPP_
#define GESTURE_CLIENT_HPP_

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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <irob_autosurg/ToolPoseStamped.h>

#include <irob_autosurg/CloseToolAction.h>
#include <irob_autosurg/OpenToolAction.h>
#include <irob_autosurg/PenetrateAction.h>
#include <irob_autosurg/GoToAction.h>

using namespace ias;

class GestureClient {

public:

   

protected:
	const std::string arm_name;
    ros::NodeHandle nh;
    
    // Action clients
    actionlib::SimpleActionClient<irob_autosurg::CloseGripperAction> 
    									 close_tool_ac("close_tool", true);
    actionlib::SimpleActionClient<irob_autosurg::OpenGripperAction>
    									 open_tool_ac("open_tool", true);
   	actionlib::SimpleActionClient<irob_autosurg::PenetrateAction> 
   										penetrate_ac("penetrate", true);
   	actionlib::SimpleActionClient<irob_autosurg::GoToAction> 
    									go_to_ac("go_to", true);
   	
   

    // States
    geometry_msgs::PoseStamped position_cartesian_current;

    // Subscribers
    ros::Subscriber position_cartesian_current_sub;



    // Publishers ?
	// ros::Publisher position_cartesian_current_pub;
   	
   	
    void subscribeTopics();
    void advertiseTopics();
    void startActionServers(); // TODO ?????

public:
	GestureClient(ros::NodeHandle, std::string);
	~GestureClient();

    // Callbacks    

    void positionCartesianCurrentCB(const geometry_msgs::PoseStampedConstPtr&);

   	Pose getPoseCurrent();
	
};


#endif /* GESTURE_SERVER_HPP_ */
