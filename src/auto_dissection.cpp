/*
 *  auto_dissection.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-02-08
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
#include "dvrk_automation/blunt_dissector.hpp"


int main(int argc, char **argv)
{
	// Initialize ros node
    ros::init(argc, argv, "irob_auto_dissection");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm;
	priv_nh.getParam("arm", arm);
	
	
	double rate_command;
	priv_nh.getParam("rate", rate_command);
	
	double speed;
	priv_nh.getParam("speed", speed);
	
	std::string regfile;
	priv_nh.getParam("regfile", regfile);

		
	double dt = 1.0/ rate_command;
	dvrk::Trajectory<double> to_enable_cartesian;
	dvrk::Trajectory<dvrk::Pose> circle_tr;
    
    // Robot control
  	try {
    	dvrk_automation::BluntDissector dissector(nh,
    			dvrk::ArmTypes::typeForString(arm), dt, regfile);
    	ros::Duration(3.0).sleep();
   	
   	 		
		// Do magic	
		
		/*
		pose: 
  position: 
    x: -0.0352589864065
    y: -0.0622652795347
    z: -0.0603390324918
  orientation: 
    x: 0.572063821814
    y: 0.703609906823
    z: -0.415625078075
    w: 0.0702273256427
*/
		
		//dissector.dissect();
		
		//dissector.toolRotate(90.0);
   	 	//ros::Duration(3.0).sleep();
   	 	//dissector.toolRotate(-90.0);		
   	 	/*dissector.toolOpen(20.0);
   	 	ros::Duration(3.0).sleep();
   	 	dissector.toolClose(10.0, 5.0);
   	 	ros::Duration(3.0).sleep();
   	 	dissector.toolOpen(20.0, 20.0);
   	 	ros::Duration(3.0).sleep();
   	 	dissector.toolClose();*/
    	//ros::Duration(120.0).sleep();
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}

