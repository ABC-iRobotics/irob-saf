/*
 * 	maneuver_client.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-20
 *
 */

#ifndef MANEUVER_CLIENT_HPP_
#define MANEUVER_CLIENT_HPP_

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

#include <irob_autosurg/GraspAction.h>
#include <irob_autosurg/DissectAction.h>

using namespace ias;

class ManeuverClient {

public:

   

protected:
	const std::string arm_name;
    ros::NodeHandle nh;
    
    // Action clients
    actionlib::SimpleActionClient<irob_autosurg::GraspAction> 
    									 grasp_ac("grasp", true);
    actionlib::SimpleActionClient<irob_autosurg::DissectAction>
    									 dissect_ac("dissect", true);
   	
   	
   

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
	ManeuverClient(ros::NodeHandle, std::string);
	~ManeuverClient();

    // Callbacks    

    void positionCartesianCurrentCB(const geometry_msgs::PoseStampedConstPtr&);

   	Pose getPoseCurrent();
	
};


#endif /* GESTURE_SERVER_HPP_ */
