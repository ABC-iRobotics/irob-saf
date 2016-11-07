/*
 * dvrk_move_test.cpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>
#include "dvrk_arm.hpp"
#include "pose.hpp"
#include "trajectory_factory.hpp"



int main(int argc, char **argv)
{

	if (argc < 5) 
	{
		std::cout << 
		"Use with params: PSM1/PSM2; init_pos/no_init_pos; rate; speed_divider" 
		<< std::endl;
		return 1;
	}
	
	std::istringstream ss1(argv[3]);
	int rate_command;
	ss1 >> rate_command;
	
	std::istringstream ss2(argv[4]);
	int speed_divider;
	ss2 >> speed_divider;	
		
	double dt = 1.0/ rate_command;
	
	
	// Initialize node
    ros::init(argc, argv, "irob_dvrk_move_test");
    ros::NodeHandle nh;
  
    DVRKArm psm(nh, DVRKArmTypes::typeForString(argv[1]));

    psm.subscribe(DVRKArmTopics::GET_ROBOT_STATE);
    psm.advertise(DVRKArmTopics::SET_ROBOT_STATE);

    psm.subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    psm.advertise(DVRKArmTopics::SET_POSITION_JOINT);

    psm.subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    psm.advertise(DVRKArmTopics::SET_POSITION_CARTESIAN);

    //psm.home(); 
       
	// Init position if necessary
	if (std::string(argv[2]) == "init_pos")
	{
		// init
		ROS_INFO("Going to init position...");
		psm.setRobotState(DVRKArm::STATE_POSITION_JOINT);
		int init_joint_idx = 2;
		Trajectory<double>* to_enable_cartesian = 
   		TrajectoryFactory::linearTrajectory(
   			psm.getJointStateCurrent(init_joint_idx), 
   			0.07, 1.0*speed_divider, dt);
		psm.playTrajectory(init_joint_idx, *to_enable_cartesian);
   	 
   		delete(to_enable_cartesian);
   	}
   	
   	// Make circles
   	ROS_INFO("Starting programmed movement...");
   	ROS_INFO("Loop rate:\t%d Hz", rate_command);
   	ROS_INFO("Speed divider:\t%d", speed_divider);
    psm.setRobotState(DVRKArm::STATE_POSITION_CARTESIAN);

    ros::Duration(1.0).sleep();

    double r = 0.02;
    
    Pose poseto( 0.0105937964763,0.0409808721132, -0.0676489437985,  -0.134924830481, 0.66488253694, 0.694115432261,0.240687076699, 0);
    
    Trajectory<Pose>* circle_tr =
    		TrajectoryFactory::linearTrajectory(
    		psm.getPoseCurrent(), 
			poseto,
			3.0*speed_divider, dt); 
    
    /*Trajectory<Eigen::Vector3d>* circle_tr =
    		TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			2*M_PI, psm.getPositionCartesianCurrent() + Eigen::Vector3d(0.0, -r, 0.0),
			3.0*speed_divider, dt);   */
   	    
   // while(ros::ok()) {
    	psm.playTrajectory(*circle_tr);
   // }	
    delete(circle_tr);
    
    //psm2.home();
    std::cout << std::endl << "Stopping program..." << std::endl;
    ros::shutdown();
	return 0;
}




