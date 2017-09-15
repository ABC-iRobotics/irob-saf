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
#include <irob_utils/pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/irob_action_client.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/GestureAction.h>

namespace ias {

class GestureClient {

protected:
	const std::string arm_name;
    ros::NodeHandle nh;
    
    // Action clients
    IrobActionClient<irob_msgs::GestureAction> ac;
   

    // States
    irob_msgs::ToolPoseStamped position_cartesian_current;
    
    // Subscribers
    ros::Subscriber position_cartesian_current_sub;

    // Publishers
	ros::Publisher position_cartesian_current_pub;
   	
   	
    void subscribeTopics();
    void advertiseTopics();
    void startActionClients(); 
    void waitForActionServer();

public:
	GestureClient(ros::NodeHandle, std::string);
	~GestureClient();

    // Callbacks    

    void positionCartesianCurrentCB(
    		const irob_msgs::ToolPoseStampedConstPtr&);


	
   	Pose getPoseCurrent();
   	std::string getName();
   	
   		// Robot motions
   	void toolClose(double, double = 10.0);	
   	void toolOpen(double, double = 10.0);
	void inTCPforward(double, double = 10.0);
	void inTCPbackward(double, double = 10.0);
	void goTo(Pose, double = 10.0, std::vector<Pose> = std::vector<Pose>(), 
			InterpolationMethod = LINEAR);
			
	bool isGestureDone();
	
};

}
#endif /* GESTURE_SERVER_HPP_ */
