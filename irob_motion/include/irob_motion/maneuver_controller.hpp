/*
 * 	maneuver_controller.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-10-16
 *
 *	Supports N arms through N GestureClients.
 *	Dissect, pass object, grasp(including goto)
 *	Redundant maneuvers like grasping and passing?
 *
 *	TODO Handle tool types, allow gestures etc?
 */

#ifndef MANEUVER_CONTROLLER_HPP_
#define MANEUVER_CONTROLLER_HPP_

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
#include <irob_motion/gesture_client.hpp>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/VisionObject.h>


namespace ias {

class ManeuverController {
public:
  

private:
	
    ros::NodeHandle nh;
    std::vector<GestureClient*> arms;
    
    // Action server
    actionlib::SimpleActionServer<irob_msgs::ManeuverAction> as;   	
   	

    // Publishers
    /*ros::Publisher robot_state_pub;
    ros::Publisher position_joint_pub;
    ros::Publisher position_cartesian_pub;

	ros::Publisher position_cartesian_current_pub;
   	*/ 
   	
    void subscribeTopics();
    void advertiseTopics();
    
    int findArmIdx(std::string);

public:
	ManeuverController(ros::NodeHandle, std::vector<std::string>);
	~ManeuverController();

    // Callbacks
    void maneuverActionCB(
    		const irob_msgs::ManeuverGoalConstPtr &);
    	
    void dissect(std::string, Pose, double, double, double, std::vector<Pose>);
    		
   	void grasp(std::string, Pose, double, double, double, std::vector<Pose>);
    		
    void moveTo(std::string, Pose, std::vector<Pose>);
    		
 	
	
};

}
#endif /* MANEUVER_CONTROLLER_HPP_ */
