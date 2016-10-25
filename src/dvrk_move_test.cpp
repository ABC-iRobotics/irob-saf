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
#include "trajectory_factory.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "irob_dvrk_move_test");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    DVRKArm psm1(nh, DVRKArmTypes::PSM1);

    psm1.subscribe(DVRKArmTopics::GET_ROBOT_STATE);
    psm1.advertise(DVRKArmTopics::SET_ROBOT_STATE);

    psm1.subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    psm1.advertise(DVRKArmTopics::SET_POSITION_JOINT);

    psm1.subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    psm1.advertise(DVRKArmTopics::SET_POSITION_CARTESIAN);

    psm1.home();
    psm1.setRobotState(DVRKArm::STATE_POSITION_JOINT);
    
    double speed_divider = 10.0;
    
	// init
	int init_joint_idx = 2;
	Trajectory<double>* to_enable_cartesian = 
   		TrajectoryFactory::linearTrajectory(
   			psm1.getJointStateCurrent(init_joint_idx), 0.07, 1.0*speed_divider, 0.05);
		psm1.playTrajectory(init_joint_idx, *to_enable_cartesian);
   	ros::Duration(0.5).sleep();
   	 
   	delete(to_enable_cartesian);
   	
   	// cartesian
    psm1.setRobotState(DVRKArm::STATE_POSITION_CARTESIAN);

    double r = 0.02;
    
    Trajectory<Vector3D>* circle_tr =
    		TrajectoryFactory::circleTrajectoryHorizontal(
    		psm1.getPositionCartesianCurrent(), 
			2*M_PI, psm1.getPositionCartesianCurrent() + Vector3D(0.0, -r, 0.0),
			5.0*speed_divider, 0.1);   
   	    
    while(ros::ok()) {
    	psm1.playTrajectory(*circle_tr);
    }	
    delete(circle_tr);
    
    psm1.home();
    std::cout << std::endl << "Stopping prigram..." << std::endl;
	return 0;
}




