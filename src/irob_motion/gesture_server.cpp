/*
 *  gesture_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-18
 *  
 */

#include "irob_motion/gesture_server.hpp"

using namespace ias;


GestureServer::GestureServer(ros::NodeHandle nh, std::string arm_name, 
													double dt): 
			nh(nh), arm(nh, arm_name, dt),
			close_tool_as(nh, "close_tool", boost::bind(
				&GestureServer::closeToolActionCB, this, _1), false),
			open_tool_as(nh, "open_tool", boost::bind(
				&GestureServer::openToolActionCB, this, _1), false),
			penetrate_as(nh,"penetrate",boost::bind(
				&GestureServer::penetrateActionCB, this, _1), false),
			go_to_as(nh,"go_to",boost::bind(
				&GestureServer::goToActionCB, this, _1), false)
{

	// Subscribe and advertise topics

    startActionServers();
    arm.initArm(true, false);    
}

GestureServer::~GestureServer()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void GestureServer::closeToolActionCB(
				const irob_autosurg::CloseToolGoalConstPtr &goal)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(0.2);

    irob_autosurg::CloseToolFeedback feedback;
    irob_autosurg::CloseToolResult result;

	ROS_INFO_STREAM(arm.getName() << " closing tool");
  	
  	arm.moveGripper(goal->angle, goal->speed);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (close_tool_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << " close tool: Preempted");
        	// Set the action state to preempted
        	close_tool_as.setPreempted();
        	success = false;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
		
		success = arm.isFollowTrajectoryDone();
  		loop_rate.sleep();
  		// TODO send feedback
    }

    if(success)
    {
      	result.angle = (arm.getPoseCurrent().jaw * 180.0) / (M_PI);
      	ROS_INFO_STREAM(arm.getName()  << " close arm succeeded");
		result.info = arm.getName()  + " close arm succeeded";
      	// set the action state to succeeded
      	close_tool_as.setSucceeded(result);
    }
}
  
void GestureServer::openToolActionCB(
				const irob_autosurg::OpenToolGoalConstPtr &goal)
{
     // Helper variables
    bool success = false;
    ros::Rate loop_rate(0.2);

    irob_autosurg::OpenToolFeedback feedback;
    irob_autosurg::OpenToolResult result;

	ROS_INFO_STREAM(arm.getName()  << " closing tool");
  	
  	arm.moveGripper(goal->angle, goal->speed);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (open_tool_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << " open tool: Preempted");
        	// Set the action state to preempted
        	open_tool_as.setPreempted();
        	success = false;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
		
		success = arm.isFollowTrajectoryDone();
  		loop_rate.sleep();
  		// TODO send feedback
    }

    if(success)
    {
      	result.angle = (arm.getPoseCurrent().jaw * 180.0) / (M_PI);
      	ROS_INFO_STREAM(arm.getName()  << " open arm succeeded");
		result.info = arm.getName()  + " open arm succeeded";
      	// set the action state to succeeded
      	open_tool_as.setSucceeded(result);
    }
}

void GestureServer::penetrateActionCB(
				const irob_autosurg::PenetrateGoalConstPtr &goal)
{
     // Helper variables
    bool success = false;
    ros::Rate loop_rate(0.2);

    irob_autosurg::PenetrateResult result;

	// TODO
  	ROS_INFO_STREAM("Penetration not implemented yet");
  	//arm.moveGripper(goal->angle, goal->speed);

    // set the action state to succeeded
    penetrate_as.setSucceeded(result);

}
  
  
void GestureServer::goToActionCB(
				const irob_autosurg::GoToGoalConstPtr &goal)
{
       // Helper variables
    bool success = false;
    ros::Rate loop_rate(0.2);

    irob_autosurg::GoToResult result;

	// TODO
  	ROS_INFO_STREAM("Go to not implemented yet");
  	//arm.moveGripper(goal->angle, goal->speed);

    // set the action state to succeeded
    go_to_as.setSucceeded(result);
}


void GestureServer::startActionServers() 
{

	close_tool_as.start();
	open_tool_as.start();
    penetrate_as.start();
    go_to_as.start();
}


// Simple relay
Pose GestureServer::getPoseCurrent()
{
 	return arm.getPoseCurrent();
}

std::string GestureServer::getArmName()
{
	return arm.getName();
}



