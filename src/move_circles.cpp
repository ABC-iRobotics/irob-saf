/*
 *  dvrk_move_test.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
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
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory_factory.hpp"



int main(int argc, char **argv)
{
	// Initialize ros node
    ros::init(argc, argv, "irob_dvrk_move_circles");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm;
	priv_nh.getParam("arm", arm);
	
	double rate_command;
	priv_nh.getParam("rate", rate_command);
	
	double speed;
	priv_nh.getParam("speed", speed);

		
	double dt = 1.0/ rate_command;
	dvrk::Trajectory<double> to_enable_cartesian;
	dvrk::Trajectory<dvrk::Pose> circle_tr;
    
    // Robot control
  	try {
    	dvrk::PSM psm(nh, dvrk::ArmTypes::typeForString(arm), dvrk::PSM::ACTIVE);
    	ros::Duration(1.0).sleep();  
   	
   		// Do preprogrammed movement
   		ROS_INFO_STREAM("Starting programmed movement...");
   		ROS_INFO_STREAM("Loop rate:\t" << rate_command << " Hz");
   		ROS_INFO_STREAM("Speed:\t"<< speed);
   	
   	 	psm.setRobotState(dvrk::PSM::STATE_POSITION_CARTESIAN);

    	ros::Duration(1.0).sleep();

    	double r = 0.02;
    	
    	//dvrk::Pose poseto(0.0271533,	0.028501,	-0.0355035,
    	//	-0.0590722,	0.65363,	0.540358,	0.526584,	0.689405); 
    
    	/*dvrk::Trajectory<dvrk::Pose> 
    		to_tr(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					poseto,
					0.5,2.0, dt)); */
	
		/*dvrk::Trajectory<dvrk::Pose> 
			back_tr(dvrk::
				TrajectoryFactory::linearTrajectoryWithSmoothAcceleration(
					poseto, psm.getPoseCurrent(),
					0.5,2.0, dt)); */
    	
    	
    	
		
		/*dvrk::Trajectory<Eigen::Vector3d> 
			back_tr(dvrk::
				TrajectoryFactory::linearTrajectoryWithSmoothAcceleration(
					circle_tr[circle_tr.size()-1], circle_tr[0],
					0.5,2.0, dt));  */
   	    
    	while(ros::ok()) {
    		dvrk::Trajectory<Eigen::Vector3d> circle_tr =
    		dvrk::TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, -r, 0.0),
			3.0/speed, dt); 
			
			psm.playTrajectory(circle_tr);
    		ros::Duration(1.0).sleep();
    		
			dvrk::Trajectory<Eigen::Vector3d> back_tr =
    		dvrk::TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			-2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, r, 0.0),
			3.0/speed, dt); 
    		
    		psm.playTrajectory(back_tr);
    		ros::Duration(1.0).sleep();
    		
    		dvrk::Trajectory<Eigen::Vector3d> circle_tr2 =
    		dvrk::TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(r, 0.0, 0.0),
			3.0/speed, dt); 
			
			psm.playTrajectory(circle_tr2);
    		ros::Duration(1.0).sleep();
    		
			dvrk::Trajectory<Eigen::Vector3d> back_tr2 =
    		dvrk::TrajectoryFactory::circleTrajectoryHorizontal(
    		psm.getPositionCartesianCurrent(), 
			-2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(-r, 0.0, 0.0),
			3.0/speed, dt); 
    		
    		psm.playTrajectory(back_tr2);
    		ros::Duration(1.0).sleep();
    		
    		/*dvrk::Trajectory<Eigen::Vector3d> circle_tr3 =
    		dvrk::TrajectoryFactory::circleTrajectoryVertical(
    		psm.getPositionCartesianCurrent(), 
			2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, 0.0, r),
			3.0/speed, dt); 
			
			psm.playTrajectory(circle_tr3);
    		ros::Duration(1.0).sleep();*/
    		
			dvrk::Trajectory<Eigen::Vector3d> back_tr3 =
    		dvrk::TrajectoryFactory::circleTrajectoryVerticalY(
    		psm.getPositionCartesianCurrent(), 
			-2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, 0.0, -r),
			3.0/speed, dt); 
    		
    		psm.playTrajectory(back_tr3);
    		ros::Duration(1.0).sleep();
    		
    		dvrk::Trajectory<Eigen::Vector3d> back_tr4 =
    		dvrk::TrajectoryFactory::circleTrajectoryVerticalX(
    		psm.getPositionCartesianCurrent(), 
			-2*M_PI, psm.getPositionCartesianCurrent() + 
			Eigen::Vector3d(0.0, 0.0, -r),
			3.0/speed, dt); 
    		
    		psm.playTrajectory(back_tr4);
    		ros::Duration(1.0).sleep();
    	}	
    
    	//psm.home();
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




