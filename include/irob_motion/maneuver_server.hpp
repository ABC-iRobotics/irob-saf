/*
 * 	maneuver_server.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-08-18
 *
 *	Supports N arms through N GestureClients.
 *	Dissect, pass object, grasp(including goto)
 *	Redundant maneuvers like grasping and passing?
 *
 *	TODO Handle tool types, allow gestures etc?
 */

#ifndef MANEUVER_SERVER_HPP_
#define MANEUVER_SERVER_HPP_

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
#include "irob_motion/gesture_client.hpp"

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_autosurg/ToolPoseStamped.h>

#include <irob_autosurg/GraspAction.h>
#include <irob_autosurg/DissectAction.h>

namespace ias {

class ManeuverServer {
public:
  

private:
	
    ros::NodeHandle nh;
    std::vector<GestureClient*> arms;
    
    // Action servers
    actionlib::SimpleActionServer<irob_autosurg::DissectAction>
    	 dissect_as;
   	actionlib::SimpleActionServer<irob_autosurg::GraspAction>
    	 grasp_as;
   	
   	

    // Publishers
    /*ros::Publisher robot_state_pub;
    ros::Publisher position_joint_pub;
    ros::Publisher position_cartesian_pub;

	ros::Publisher position_cartesian_current_pub;
   	*/ 
   	
    void subscribeTopics();
    void advertiseTopics();
    void startActionServers();
    
    int findArmIdx(std::string);

public:
	ManeuverServer(ros::NodeHandle, std::vector<std::string>);
	~ManeuverServer();

    // Callbacks
    void dissectActionCB(
    		const irob_autosurg::DissectGoalConstPtr &);
    		
   	void graspActionCB(
    		const irob_autosurg::GraspGoalConstPtr &);
    		
 	
	
};

}
#endif /* GESTURE_SERVER_HPP_ */
