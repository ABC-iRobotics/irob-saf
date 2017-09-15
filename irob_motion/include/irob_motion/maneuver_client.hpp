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
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <irob_utils/pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/irob_action_client.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/ManeuverAction.h>

namespace ias {

class ManeuverClient {

public:

   

protected:
	std::vector<std::string> arm_names;
    ros::NodeHandle nh;
    
    // Action clients
    IrobActionClient<irob_msgs::ManeuverAction> ac;
   	
   	// States
    std::map<std::string, irob_msgs::ToolPoseStamped>
    											position_cartesian_current;
    
    // Subscribers
    std::map<std::string, ros::Subscriber> position_cartesian_current_subs;



   	
    void subscribeTopics();
    void startActionClients(); 
    void waitForActionServer();

public:
	ManeuverClient(ros::NodeHandle, std::vector<std::string>);
	~ManeuverClient();

    // Callbacks 
    void positionCartesianCurrentCB(
    		const irob_msgs::ToolPoseStampedConstPtr&, const std::string&);   
   	
   	// Robot motions
   	void dissect(std::string, Pose, double, double, double, 
   									std::vector<Pose> = std::vector<Pose>());	
   	void grasp(std::string, Pose, double, double, double,
   									std::vector<Pose> = std::vector<Pose>());
   	void moveTo(std::string, Pose, std::vector<Pose> = std::vector<Pose>());	

			
	bool isManeuverDone();
	
	Pose getPoseCurrent(std::string);
};

}
#endif /* MANEUVER_CLIENT_HPP_ */