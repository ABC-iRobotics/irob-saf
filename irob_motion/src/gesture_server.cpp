/*
 *  gesture_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-18
 *  
 */

#include <irob_motion/gesture_server.hpp>

namespace ias {


GestureServer::GestureServer(ros::NodeHandle nh, std::string arm_name, 
													double dt): 
			nh(nh), arm(nh, arm_name, dt),
			as(nh, "gesture/"+arm_name, boost::bind(
				&GestureServer::gestureActionCB, this, _1), false)
{

	// Subscribe and advertise topics

    as.start();
    arm.initArm(true, false);    
}

GestureServer::~GestureServer()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void GestureServer::gestureActionCB(
				const irob_msgs::GestureGoalConstPtr &goal)
{
    switch(goal -> action)
    {
    	case irob_msgs::GestureGoal::GO_TO:
    	{
    		std::vector<Pose> waypoints;
			for (irob_msgs::ToolPose p : goal->waypoints)
				waypoints.push_back(Pose(p));
			InterpolationMethod	interp_method;
			if (goal -> interpolation == goal -> INTERPOLATION_LINEAR)
				interp_method = InterpolationMethod::LINEAR;
			else
				interp_method = InterpolationMethod::BEZIER;
				
    		goTo(goal -> target, goal -> speed, waypoints, interp_method);
    		break;
    	}	
    		
    	case irob_msgs::GestureGoal::TOOL_CLOSE:
    	{
    		toolClose(goal -> angle, goal -> speed);
    		break;
    	}	
    		
    	case irob_msgs::GestureGoal::TOOL_OPEN:
    	{
    		toolOpen(goal -> angle, goal -> speed);
    		break;
    	}	
    		
    	case irob_msgs::GestureGoal::IN_TCP_FORWARD:
    	{
    		inTCPforward(goal -> distance, goal -> speed);
    		break;
    	}	
    		
    	case irob_msgs::GestureGoal::IN_TCP_BACKWARD:
    	{
    		inTCPbackward(goal -> distance, goal -> speed);
    		break;
    	}	
    		
    	default:
    	{
    		ROS_ERROR_STREAM(arm.getName()  << 
    					": invalid gesture action code received");
    		irob_msgs::GestureResult result;
    		result.pose = arm.getPoseCurrent().toRosToolPose();
			result.info = arm.getName()  + 
						": invalid gesture action code";
      		as.setAborted(result);
    		break;
    	}    	
    }	
}



/**
 * Actions
 */
 
void GestureServer::toolClose(double angle, double speed)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureFeedback feedback;
    irob_msgs::GestureResult result;

	Pose p = arm.getPoseCurrent();
	double curr_angle = radToDeg(p.jaw);
	
	// Check if closing is possible
	if (curr_angle < (angle - 0.1)) {
		result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  
      			<< ": close tool not possible, goal > current");
		result.info = arm.getName()  
				+ ": close tool not possible, goal > current";
      	as.setAborted(result);
      	return;
    } 
	
	ROS_INFO_STREAM(arm.getName() << " closing tool to " << 
						angle << " degrees");
  	
  	arm.moveGripper(angle, speed);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok()) {
      		
        	ROS_INFO_STREAM(arm.getName()  << ": close tool: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
	
		
		success = arm.isFollowTrajectoryDone();
  		loop_rate.sleep();
  		// TODO send feedback
    }

    if(success)
    {
      	result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  << ": close tool succeeded");
		result.info = arm.getName()  + ": close tool succeeded";
      	// set the action state to succeeded
      	as.setSucceeded(result);
	}
	
}
  
void GestureServer::toolOpen(double angle, double speed)
{
      // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureFeedback feedback;
    irob_msgs::GestureResult result;

	Pose p = arm.getPoseCurrent();
	double curr_angle = radToDeg(p.jaw);
	
	// Check if opening is possible
	if (curr_angle > (angle + 0.1)) {
		result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  
      			<< ": open tool not possible, goal > current");
		result.info = arm.getName()  
				+ ": open tool not possible, goal > current";
      	as.setAborted(result);
      	return;
    } 
	
	ROS_INFO_STREAM(arm.getName() << " opening tool to " << 
						angle << " degrees");
  	
  	arm.moveGripper(angle, speed);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok()) {
      		
        	ROS_INFO_STREAM(arm.getName()  << ": open tool: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
	
		
		success = arm.isFollowTrajectoryDone();
  		loop_rate.sleep();
  		// TODO send feedback
    }

    if(success)
    {
      	result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  << ": open tool succeeded");
		result.info = arm.getName()  + ": open tool succeeded";
      	// set the action state to succeeded
      	as.setSucceeded(result);
	}
}

void GestureServer::inTCPforward(double distance, double speed)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureResult result;
    irob_msgs::GestureFeedback feedback;
    Eigen::Vector3d v(0.0, 0.0, distance);

	arm.moveRelative(v, speed, RobotClient::CoordFrame::TCPF);
	
	while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << ": in TCP forward: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
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
      	ROS_INFO_STREAM(arm.getName()  << ": in TCP forward succeeded");
		result.info = arm.getName()  + ": in TCP forward succeeded";
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
}

void GestureServer::inTCPbackward(double distance, double speed)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureResult result;
    irob_msgs::GestureFeedback feedback;
    Eigen::Vector3d v(0.0, 0.0, (-1.0) * distance);

	arm.moveRelative(v, speed, RobotClient::CoordFrame::TCPF);
	
	while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << ": in TCP backward: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
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
      	ROS_INFO_STREAM(arm.getName()  << ": in TCP backward succeeded");
		result.info = arm.getName()  + ": in TCP backward succeeded";
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
}
  
  
void GestureServer::goTo(Pose target, double speed, std::vector<Pose> waypoints, 
							InterpolationMethod interp_method)
{
       // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureResult result;
    irob_msgs::GestureFeedback feedback;

	ROS_INFO_STREAM(arm.getName()  << ": go to position");
	
  	arm.goTo(target, speed, waypoints, interp_method);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << ": go to: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
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
      	ROS_INFO_STREAM(arm.getName()  << ": go to succeeded");
		result.info = arm.getName()  + ": go to succeeded";
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
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


