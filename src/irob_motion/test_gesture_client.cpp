/*
 *  move_circles_action_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-21
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>

#include <irob_autosurg/CloseToolAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "irob_home_arm");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm_name;
	priv_nh.getParam("arm_name", arm_name);

	
    
    // Robot control
  	try {
			
		actionlib::SimpleActionClient<irob_autosurg::CloseToolAction>
				 close_tool_ac("close_tool", true);



		ROS_INFO("Waiting for action server to start.");
  		// wait for the action server to start
  		close_tool_ac.waitForServer(); //will wait for infinite time

 		ROS_INFO("Action server started, sending goal.");
  		// send a goal to the action
  		irob_autosurg::CloseToolGoal goal;
 
  		goal.angle = 40.0;
  		goal.speed = 10.0;
  		close_tool_ac.sendGoal(goal);
  
  

  		//wait for the action to return
  		bool finished_before_timeout = 
  				close_tool_ac.waitForResult(ros::Duration(30.0));
   

 		if (finished_before_timeout)
  		{
    		actionlib::SimpleClientGoalState state = close_tool_ac.getState();
    		ROS_INFO("Action finished: ");
  		}
  		else
   		 	ROS_INFO("Action did not finish before the time out.");
   	
    		ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    	} catch (const std::exception& e) {
  			ROS_ERROR_STREAM(e.what());
  			ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  		}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




