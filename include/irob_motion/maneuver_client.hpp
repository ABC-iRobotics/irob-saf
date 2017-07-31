/*
 * 	maneuver_client.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-31
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
#include "irob_utils/irob_action_client.hpp"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_autosurg/ToolPoseStamped.h>

#include <irob_autosurg/DissectAction.h>
#include <irob_autosurg/GraspAction.h>

using namespace ias;

class ManeuverClient {

public:

   

protected:
	const std::string arm_name;
    ros::NodeHandle nh;
    
    // Action clients
    IrobActionClient<irob_autosurg::DissectAction> 
    									 dissect_ac;
    IrobActionClient<irob_autosurg::GraspAction>
    									 grasp_ac;
   	
   	
   	
    void subscribeTopics();
    void advertiseTopics();
    void startActionClients(); 
    void waitForActionServers();

public:
	ManeuverClient(ros::NodeHandle);
	~ManeuverClient();

    // Callbacks    
   	
   	// Robot motions
   	void dissect(std::string, Pose, double, double, double);	
   	void grasp(std::string, Pose, double, double);	

			
	bool isDissectDone();
	bool isGraspDone();	
};


#endif /* GESTURE_SERVER_HPP_ */
