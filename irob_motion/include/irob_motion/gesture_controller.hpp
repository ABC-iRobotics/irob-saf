/*
 * 	gesture_controller.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-10-16
 *	
 *	grasp, release, cut, go_to, approach, leave...
 */

#ifndef GESTURE_CONTROLLER_HPP_
#define GESTURE_CONTROLLER_HPP_

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

#include <irob_msgs/VisionObject.h>



namespace ias {

class GestureController {

public:

   

protected:
	RobotClient arm;
    ros::NodeHandle nh;
	

public:
	GestureController(ros::NodeHandle, std::string, double);		// dt
	~GestureController();

    // Callbacks

    void gestureActionCB(
    		const irob_msgs::GestureGoalConstPtr &);
   
   	void closeJaws(double, double); 	
   	
   	void openJaws(double, double);
    		
   	void approach(double, double);
   	
    void leave(double, double);
    		
   	void goTo(Pose, double,	std::vector<Pose>, InterpolationMethod);
    
    void stop(); 

   	Pose getPoseCurrent();
   	std::string getArmName();	
};

}
#endif /* GESTURE_CONTROLLER_HPP_ */
