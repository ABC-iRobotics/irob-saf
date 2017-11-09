/*
 * 	gesture_server.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-06
 *	
 *	Separated ROS node to support preempted actions.
 *	grasp, release, cut, go_to, approach, leave...
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
    
    static const double DEFAULT_SPEED_CARTESIAN;	// mm/s
    static const double DEFAULT_SPEED_JAW;			// deg/s
    static const double DEFAULT_LOOP_RATE;					// Hz
	

public:
	GestureServer(ros::NodeHandle, std::string, double);		// dt
	~GestureServer();

    // Callbacks

    void gestureActionCB(
    		const irob_msgs::GestureGoalConstPtr &);
    		
    Pose getPoseCurrent();
   	std::string getArmName();	
    		
protected:
		
	// Methods for gesture execution
	void stop(); 
	  
	void standby(Pose ,std::vector<Pose>, InterpolationMethod, double); 
	 
   	void grasp(Pose, Pose, double, double, std::vector<Pose>,
   			InterpolationMethod, 
   			double, double);
   	
   	void cut(Pose, Pose, double, double, std::vector<Pose>, 
   			InterpolationMethod,
   			double, double);
   			
   	void push(Pose, Pose, Eigen::Vector3d, double, std::vector<Pose>,
	  		InterpolationMethod,
	  		double, double);
    	
    void dissect(Pose, Pose, Eigen::Vector3d,double, double, std::vector<Pose>,
    		InterpolationMethod,
    		double, double); 
    		
   	void release(Pose, double, 
   			double, double);
   	
   	void place(Pose, Pose, std::vector<Pose>, InterpolationMethod,
   			double);
	
   	void manipulate(Eigen::Vector3d,
   			double);
   			
    bool waitForActionDone(std::string);	
	bool handleActionState(std::string, bool = false);
   	
};

}
#endif /* GESTURE_SERVER_HPP_ */
