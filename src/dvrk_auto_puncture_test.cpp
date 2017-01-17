/*
 *  dvrk_auto_puncture_test.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-03
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <numeric>
#include <chrono>
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory_factory.hpp"
#include "dvrk_automation/puncturer.hpp"


int main(int argc, char **argv)
{

	// Check command line arguments
	if (argc < 12) 
	{
		std::cout << 
		"Use with params (in s and mm): PSM1/PSM2; rate; filename_base; "
		<< "area_x; area_y; nlocs_x; nlocs_y; ntrials; depth; speed; T" 
		<< std::endl;
		return 1;
	}
	
	std::istringstream ss1(argv[2]);
	int rate_command;
	ss1 >> rate_command;
	
	double dt = 1.0/ rate_command;

	
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_auto_puncture_test");
    ros::NodeHandle nh;
    
    // Robot control
  	try {
    	dvrk_automation::Puncturer punct(nh, 
    			dvrk::ArmTypes::typeForString(argv[1]),
    			dt);
    	ros::Duration(1.0).sleep();
   	
   		// Parse arguments
   		std::string fileNameBase = argv[3];
   		
   		ss1.str(argv[4]);
   		ss1.clear();
   		double area_x;
   		ss1 >> area_x;
   		
   		ss1.str(argv[5]);
   		ss1.clear();
   		double area_y;
   		ss1 >> area_y;
   		
   		Eigen::Vector2d scanningArea(area_x / 1000.0, area_y / 1000.0);
   		
   		ss1.str(argv[6]);
   		ss1.clear();
   		int nlocs_x;
   		ss1 >> nlocs_x;
   		
   		ss1.str(argv[7]);
   		ss1.clear();
   		int nlocs_y;
   		ss1 >> nlocs_y;
   		
   		Eigen::Vector2i nLocations(nlocs_x, nlocs_y);
   		
   		ss1.str(argv[8]);
   		ss1.clear();
   		int ntrials;
   		ss1 >> ntrials;
   		
   		ss1.str(argv[9]);
   		ss1.clear();
   		double depth;
   		ss1 >> depth;
   		depth /= 1000.0;
   		
   		ss1.str(argv[10]);
   		ss1.clear();
   		double speed;
   		ss1 >> speed;
   		speed /= 1000.0;
   		
   		ss1.str(argv[11]);
   		ss1.clear();
   		double T;
   		ss1 >> T;
   		
   	 		
		// Do puncture series			
   	 	punct.doPunctureSeries(scanningArea,
					nLocations, ntrials,
					depth, speed, T, 
					fileNameBase);
    
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




