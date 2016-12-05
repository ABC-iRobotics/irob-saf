/*
 *  dvrk_move_test_translate.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-05
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <numeric>
#include "optoforce/optoforce_listener.hpp"


int main(int argc, char **argv)
{
	// Initialize ros node
    ros::init(argc, argv, "optoforce_helper");
    ros::NodeHandle nh;
    	
	double dt = 1.0/ 10.0;

	ros::Rate loop_rate(1.0/dt);
	optoforce::OptoforceListener oforce(nh);
	ros::Duration(1.0).sleep();
	oforce.calibrateOffsets();
	while(ros::ok())
	{
		ROS_INFO_STREAM(oforce.getForcesCurrent().transpose());
	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




