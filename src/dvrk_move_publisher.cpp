#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>

#include <sstream>

/*
 * Globals
 */
std_msgs::String robot_state;
sensor_msgs::JointState joint_state;


/*
 * Callbacks
 */
void robot_state_cb(const std_msgs::String msg) {
	robot_state = msg;
}

void joint_state_cb(const sensor_msgs::JointStateConstPtr& msg) {
	joint_state = *msg;
	ROS_INFO("Joint state:\n");
	for (int i = 0; i < 7; i++)
		ROS_INFO("\tJoint %d:\t%f\n", i, joint_state.position[i]);
}

/*
 * DVRK actions
 */
bool setStateHome(ros::Publisher& robot_state_pub) {
	while (true) {
		std_msgs::String msg;
    	std::stringstream ss;
    	ss << "Home";
    	msg.data = ss.str();
    	robot_state_pub.publish(msg);
    	ros::Duration(0.5).sleep();
    	
    	if (robot_state.data =="DVRK_READY") {
    		ROS_INFO("State set to Home");
    		return true;
    	} else {
    		//ROS_INFO("State set to %s\n", robot_state.data.c_str());
    	}

    	ros::spinOnce();
	}
	return false;
}

bool setStatePosJoint(ros::Publisher& robot_state_pub) {
	while (true) {
		std_msgs::String msg;
    	std::stringstream ss;
    	ss << "DVRK_POSITION_JOINT";
    	msg.data = ss.str();
    	robot_state_pub.publish(msg);
    	ros::Duration(0.5).sleep();
    	
    	if (robot_state.data =="DVRK_POSITION_JOINT") {
    		ROS_INFO("State set to DVRK_POSITION_JOINT");
    		return true;
    	} else {
    		//ROS_INFO("State set to %s\n", robot_state.data.c_str());
    	}

    	ros::spinOnce();
	}
	return false;
}

void moveJointRelative(ros::Publisher& joint_state_pub, 
							int joint_idx, double movement)
{
	ros::spinOnce();
	sensor_msgs::JointState new_joint_state = joint_state;
	// TODO safety
	new_joint_state.position[joint_idx] += movement;
	joint_state_pub.publish(new_joint_state);
	ros::spinOnce();
	ros::Duration(0.1).sleep();
}

/*
 * Main
 */
int main(int argc, char **argv)
{
  	ros::init(argc, argv, "irob_dvrk_move_publisher");
  	ros::NodeHandle nh;
	
	// Publishers
  	ros::Publisher robot_state_pub = nh.advertise<std_msgs::String>(
  								"/dvrk/PSM1/set_robot_state",
  								1000
  								);
  								
  	ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>(
  								"/dvrk/PSM1/set_position_joint",
  								1000
  								);
  	
  	
  	// Subscribers
  	ros::Subscriber robot_state_sub = nh.subscribe<std_msgs::String>(
  								"/dvrk/PSM1/robot_state",
  								1000,
  								robot_state_cb
  								);
  								
  	ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>(
  								"/dvrk/PSM1/state_joint_current",
  								1000,
  								joint_state_cb
  								);

  	ros::Rate loop_rate(10);
	
	setStateHome(robot_state_pub);
	setStatePosJoint(robot_state_pub);
	double increment = 0.01;
	for (double p = 0.0; p <= 1.0; p+= increment)
	{
		moveJointRelative(joint_state_pub, 0, increment);
	}
	ros::spin();
  	return 0;
}
