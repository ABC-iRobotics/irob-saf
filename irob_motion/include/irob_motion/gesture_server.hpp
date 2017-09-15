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
#include <irob_utils/pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_general_robot/robot_client.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/GestureAction.h>


namespace ias {

class GestureServer {

public:

   

protected:
	RobotClient arm;
    ros::NodeHandle nh;
    
    // Action servers
    actionlib::SimpleActionServer<irob_msgs::GestureAction> as;
	

public:
	GestureServer(ros::NodeHandle, std::string, double);		// dt
	~GestureServer();

    // Callbacks

    void gestureActionCB(
    		const irob_msgs::GestureGoalConstPtr &);
   
   	void toolClose(double, double); 	
    		
   	void toolOpen(double, double);
    		
   	void inTCPforward(double, double);
   	
    void inTCPbackward(double, double);
    		
   	void goTo(Pose, double,	std::vector<Pose>, InterpolationMethod);
    

   	Pose getPoseCurrent();
   	std::string getArmName();	
};

}
#endif /* GESTURE_SERVER_HPP_ */
