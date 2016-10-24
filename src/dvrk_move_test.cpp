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

	//ros::spin();
    double increment = 0.005;
    for (double p = 0.0; p <= 0.07; p+= increment)
    {
        psm1.moveJointRelative(2, increment);
    }

    psm1.setRobotState(DVRKArm::STATE_POSITION_CARTESIAN);

    double rad_increment = 0.1;
    double r = 0.02;
    std::vector<double> pos1;
    		pos1.push_back(sin(0.0)*r);
    		pos1.push_back(cos(0.0)*r);
    		pos1.push_back(-0.068558);
   	     	psm1.moveCartesianAbsolute(pos1);
   	     	ros::Duration(1.0).sleep();
    while(ros::ok()) {
    	for (double ang = 0.0; ang <= 2.0*M_PI && ros::ok(); ang+= rad_increment)
    	{
   			std::vector<double> pos;
    		pos.push_back(sin(ang)*r);
    		pos.push_back(cos(ang)*r);
    		pos.push_back(-0.068558);
   	     	psm1.moveCartesianAbsolute(pos);
   	     	ros::spinOnce();
    	}
    }	
    std::cout << "Stopping prigram..." << std::endl;
	return 0;
}




