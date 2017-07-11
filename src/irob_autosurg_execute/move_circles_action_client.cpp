/*
 *  move_circles_action_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-03-07-08
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
#include "irob_dvrk/arm.hpp"
#include "irob_dvrk/psm.hpp"
#include "irob_utils/pose.hpp"
#include "irob_utils/trajectory_factory.hpp"
#include <irob_autosurg/InitArmAction.h>
#include <irob_autosurg/ResetPoseAction.h>
#include <irob_autosurg/FollowTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace irob_autosurg;

int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "irob_home_arm");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm_name;
	priv_nh.getParam("arm", arm_name);
	
	double rate_command;
	priv_nh.getParam("rate", rate_command);
	
	double speed;
	priv_nh.getParam("speed", speed);

		
	double dt = 1.0/ rate_command;

	
    
    // Robot control
  	try {

    	 
    	 Pose curr(2.33248524057e-07, 2.66674690499e-07, -0.063499999999,
    	 	 -2.59734823723e-06, 0.707106781172, 0.707106781191, 
    	 	 2.59734823723e-06,
    	 	 0.0);
    	 double r = 0.02;
    	 
    	 Trajectory<Pose> circle_tr =
    	 	TrajectoryFactory::circleTrajectoryHorizontal(
    		curr, 
			2*M_PI, curr.position + 
			Eigen::Vector3d(0.0, -r, 0.0),
			3.0/speed, dt);
			
			actionlib::SimpleActionClient<irob_autosurg::InitArmAction> init_arm_ac("init_arm", true);
			actionlib::SimpleActionClient<irob_autosurg::FollowTrajectoryAction> ac("follow_trajectory", true);


	ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  init_arm_ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  irob_autosurg::InitArmGoal init_goal;
 
  init_goal.move_allowed = true;
  init_arm_ac.sendGoal(init_goal);
  
  /*

  //wait for the action to return
  bool finished_before_timeout = init_arm_ac.waitForResult(ros::Duration(30.0));
    	
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  irob_autosurg::FollowTrajectoryGoal goal;
  int s = circle_tr.size();
  for (int i = 0; i < s; i++)
  	goal.trajectory.poses.push_back(circle_tr[i].toRosToolPose());
  goal.trajectory.dt = dt;
  ac.sendGoal(goal);

  //wait for the action to return
  finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: ");
  }
  else
    ROS_INFO("Action did not finish before the time out.");
   */ 	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




