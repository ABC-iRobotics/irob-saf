/*
 *  dvrk_rviz_plot_trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-26
 *  
 */

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/trajectory_factory.hpp"



int main(int argc, char **argv)
{
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_rviz_plot_trajectory");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm;
	priv_nh.getParam("arm", arm);
	
	double rate_command;
	priv_nh.getParam("rate", rate_command);
	
	 std::string frame_id;
	priv_nh.getParam("frame_id", frame_id);

		
	double dt = 1.0/ rate_command;


    dvrk::Arm psm(nh, dvrk::ArmTypes::typeForString(arm), dvrk::Arm::PASSIVE);
    
	ROS_INFO_STREAM("Start plotting trajectory...");
	
	ros::Publisher vis_pub = 
		nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	
	ros::Rate loop_rate(1.0/dt);


	try	{
		
		int i = 0;
		while (ros::ok())
		{
			dvrk::Pose curr_pos = psm.getPoseCurrent();
			
			visualization_msgs::Marker marker;
			marker.header.frame_id = frame_id;
			marker.header.stamp = ros::Time();
			marker.ns = "dvrk_viz";
			marker.id = i++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = curr_pos.position.x();
			marker.pose.position.y = curr_pos.position.y();
			marker.pose.position.z = curr_pos.position.z();
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.002;
			marker.scale.y = 0.002;
			marker.scale.z = 0.002;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			
			vis_pub.publish( marker );
			
			loop_rate.sleep();
		}
	} catch (const std::exception& e) {
  		std::cerr << e.what() << std::endl;
  	}
   	
   	std::cout << std::endl << "Program finished succesfully, shutting down ..."
   		 << std::endl;
   	
   	ros::shutdown();
	return 0;
}






