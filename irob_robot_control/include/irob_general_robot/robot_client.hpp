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
#include <string>
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
#include <irob_utils/trajectory_factory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/irob_action_client.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/RobotAction.h>

namespace ias {

class RobotClient {

public:
	typedef enum CoordFrame 
    	{WCS, TCPF} CoordFrame;  
   

private:
	const std::string arm_name;
    ros::NodeHandle nh;
    
    double dt;
    
    // Action clients
    IrobActionClient<irob_msgs::RobotAction> ac;
   

    // States
    irob_msgs::ToolPoseStamped position_cartesian_current;

    // Subscribers
    ros::Subscriber position_cartesian_current_sub;



    // Publishers
	ros::Publisher position_cartesian_current_pub;
   	
    void subscribeTopics();
    void advertiseTopics();
    void waitForActionServer();
    

public:
	RobotClient(ros::NodeHandle, std::string, double);
	~RobotClient();

    // Callbacks    

    void positionCartesianCurrentCB(
    		const irob_msgs::ToolPoseStampedConstPtr&);

   	Pose getPoseCurrent();
   	std::string getName();
   	
   	// Robot motions
   	void initArm(bool);	
   	void resetPose(bool);
	void moveJaws(double, double = 10.0);
	void moveTool(Pose, double = 10.0, std::vector<Pose> = std::vector<Pose>(), 
			InterpolationMethod = LINEAR);
	
	void stop();
	
	
	bool isActionDone();
	
};

}
#endif /* ROBOT_CLIENT_HPP_ */
