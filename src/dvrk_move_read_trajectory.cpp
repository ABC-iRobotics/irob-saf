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
#include <fstream>
#include <string>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "irob_dvrk_move_test");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    DVRKArm psm2(nh, DVRKArmTypes::PSM2);

    psm2.subscribe(DVRKArmTopics::GET_ROBOT_STATE);
    psm2.advertise(DVRKArmTopics::SET_ROBOT_STATE);

    psm2.subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    psm2.advertise(DVRKArmTopics::SET_POSITION_JOINT);

    psm2.subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    psm2.advertise(DVRKArmTopics::SET_POSITION_CARTESIAN);

    //psm2.home();
    Trajectory<Vector3D> tr(0.1);
    
    std::ifstream logfile (argv[1]);
    if (logfile.is_open())
  	{
    	while (!logfile.eof() )
    	{
    		Vector3D v;
      		logfile >> v;
      		tr.addPoint(v);
      		//std::cout << v << std::endl;
    	}
    	logfile.close();

   	// cartesian
    psm2.setRobotState(DVRKArm::STATE_POSITION_CARTESIAN);

	Trajectory<Vector3D>* to_start = 
   		TrajectoryFactory::linearTrajectory(
   			psm2.getPositionCartesianCurrent(), tr[0], 2.0, 0.1);
		psm2.playTrajectory(*to_start);
   	ros::Duration(0.5).sleep();
   	 
   	delete(to_start);	
	
    psm2.playTrajectory(tr);
    
    }
   	else std::cout << "Unable to open file";
    //psm2.home();
    std::cout << std::endl << "Stopping prigram..." << std::endl;
	return 0;
}




