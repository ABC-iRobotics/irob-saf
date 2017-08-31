/*
 *  gesture_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-18
 *  
 */

#include "irob_motion/gesture_server.hpp"

namespace ias {


GestureServer::GestureServer(ros::NodeHandle nh, std::string arm_name, 
													double dt): 
			nh(nh), arm(nh, arm_name, dt),
			close_tool_as(nh, "gesture/"+arm_name+"/close_tool", boost::bind(
				&GestureServer::closeToolActionCB, this, _1), false),
			open_tool_as(nh, "gesture/"+arm_name+"/open_tool", boost::bind(
				&GestureServer::openToolActionCB, this, _1), false),
			push_in_as(nh,"gesture/"+arm_name+"/push_in",boost::bind(
				&GestureServer::pushInActionCB, this, _1), false),
			pull_out_as(nh,"gesture/"+arm_name+"/pull_out",boost::bind(
				&GestureServer::pullOutActionCB, this, _1), false),
			go_to_as(nh,"gesture/"+arm_name+"/go_to",boost::bind(
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
    ros::Rate loop_rate(50.0);

    irob_autosurg::CloseToolFeedback feedback;
    irob_autosurg::CloseToolResult result;

	Pose p = arm.getPoseCurrent();
	double curr_angle = radToDeg(p.jaw);
	
	if (curr_angle < (goal->angle - 0.1)) {
		result.angle = curr_angle;
      	ROS_INFO_STREAM(arm.getName()  
      			<< " close tool not possible, goal > current");
		result.info = arm.getName()  
				+ " close tool not possible, goal > current";
      	close_tool_as.setAborted(result);
      	return;
    } 
	
	ROS_INFO_STREAM(arm.getName() << " closing tool to " << 
						goal->angle << " degrees");
  	
  	arm.moveGripper(goal->angle, goal->speed);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (close_tool_as.isPreemptRequested() || !ros::ok()) {
      		
        	ROS_INFO_STREAM(arm.getName()  << " close tool: Preempted");
        	// Set the action state to preempted
        	close_tool_as.setPreempted();
        	success = false;
        	break;
      	}
	
		
		success = arm.isFollowTrajectoryDone();
  		loop_rate.sleep();
  		// TODO send feedback
    }

    if(success)
    {
      	result.angle = radToDeg(arm.getPoseCurrent().jaw);
      	ROS_INFO_STREAM(arm.getName()  << " close tool succeeded");
		result.info = arm.getName()  + " close tool succeeded";
      	// set the action state to succeeded
      	close_tool_as.setSucceeded(result);
	}
	
}
  
void GestureServer::openToolActionCB(
				const irob_autosurg::OpenToolGoalConstPtr &goal)
{
     // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::OpenToolFeedback feedback;
    irob_autosurg::OpenToolResult result;
    
    Pose p = arm.getPoseCurrent();
	double curr_angle = radToDeg(p.jaw);
	
	if (curr_angle > (goal->angle + 0.1)) {
		result.angle = curr_angle;
      	ROS_INFO_STREAM(arm.getName()  
      			<< " open tool not possible, goal < current");
		result.info = arm.getName()  + " open tool not possible, goal < current";
      	open_tool_as.setAborted(result);
    	return;
	}
	
	ROS_INFO_STREAM(arm.getName()  << " opening tool to " 
			<< goal->angle << " degrees");
  	
  	
  	
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
    	result.angle = radToDeg(arm.getPoseCurrent().jaw);
      	ROS_INFO_STREAM(arm.getName()  << " open tool succeeded");
		result.info = arm.getName()  + " open tool succeeded";
      	// set the action state to succeeded
      	open_tool_as.setSucceeded(result);
    }
}

void GestureServer::pushInActionCB(
				const irob_autosurg::PushInGoalConstPtr &goal)
{
     // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::PushInResult result;
    Eigen::Vector3d v(0.0, 0.0, (goal->depth));

	arm.moveRelative(v,  goal->speed, RobotClient::CoordFrame::TCPF);
	
	while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (open_tool_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << " push in: Preempted");
        	// Set the action state to preempted
        	push_in_as.setPreempted();
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
      	ROS_INFO_STREAM(arm.getName()  << " push in succeeded");
		result.info = arm.getName()  + " push in succeeded";
      	// set the action state to succeeded
      	push_in_as.setSucceeded(result);
    }
}

void GestureServer::pullOutActionCB(
				const irob_autosurg::PullOutGoalConstPtr &goal)
{
     // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::PullOutResult result;
 	Eigen::Vector3d v(0.0, 0.0, -(goal->depth));

	arm.moveRelative(v,  goal->speed, RobotClient::CoordFrame::TCPF);
	
	while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (open_tool_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << " pull out: Preempted");
        	// Set the action state to preempted
        	pull_out_as.setPreempted();
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
      	ROS_INFO_STREAM(arm.getName()  << " pull out succeeded");
		result.info = arm.getName()  + " pull out succeeded";
      	// set the action state to succeeded
      	pull_out_as.setSucceeded(result);
    }
}
  
  
void GestureServer::goToActionCB(
				const irob_autosurg::GoToGoalConstPtr &goal)
{
       // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::GoToResult result;
    irob_autosurg::GoToFeedback feedback;

	ROS_INFO_STREAM(arm.getName()  << " go to pos");
	
	Pose target(goal->target);
	std::vector<Pose> waypoints;
	for (irob_autosurg::ToolPose p : goal->waypoints)
		waypoints.push_back(Pose(p));
  	arm.goTo(target, goal->speed, waypoints, InterpolationMethod::LINEAR);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (go_to_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << " go to: Preempted");
        	// Set the action state to preempted
        	go_to_as.setPreempted();
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
    	result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  << " go to succeeded");
		result.info = arm.getName()  + " go to succeeded";
      	// set the action state to succeeded
      	go_to_as.setSucceeded(result);
    }
}


void GestureServer::startActionServers() 
{
	close_tool_as.start();
	open_tool_as.start();
    push_in_as.start();
    pull_out_as.start();
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

}

using namespace ias;


/**
 * Gesture server main 
 */
int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "gesture_server");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
	
	std::string arm_name;
	priv_nh.getParam("arm_name", arm_name);
	
	double rate_command;
	priv_nh.getParam("rate", rate_command);
	
    
    // StartGesture server
  	try {
    	GestureServer gesture(nh, arm_name, 1.0/rate_command);
    	
  	   	ros::spin();	    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}


