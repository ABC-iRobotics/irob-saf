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
    arm.initArm(true);    
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
    		
    	case irob_msgs::GestureGoal::GRASP:
    	{
    		grasp(goal -> angle, goal -> speed);
    		break;
    	}
    	
    	case irob_msgs::GestureGoal::CUT:
    	{
    		cut(goal -> angle, goal -> speed);
    		break;
    	}	
    		
    	case irob_msgs::GestureGoal::RELEASE:
    	{
    		release(goal -> angle, goal -> speed);
    		break;
    	}
    	
    	case irob_msgs::GestureGoal::OPEN_JAWS:
    	{
    		openJaws(goal -> angle, goal -> speed);
    		break;
    	}	
    		
    	case irob_msgs::GestureGoal::APPROACH:
    	{
    		approach(goal -> movement, goal -> speed);
    		break;
    	}	
    		
    	case irob_msgs::GestureGoal::LEAVE:
    	{
    		leave(goal -> movement, goal -> speed);
    		break;
    	}
    	
    	case irob_msgs::GestureGoal::STOP:
    	{
    		stop();
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
void GestureServer::stop()
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureFeedback feedback;
    irob_msgs::GestureResult result;
	
	// Start action
	ROS_INFO_STREAM(arm.getName() << " stopping");
  	feedback.pose = arm.getPoseCurrent().toRosToolPose();
	feedback.info = "starting stop action";
    as.publishFeedback(feedback);
    
  	arm.stop();
  	
  	// Wait for action to be done
    while(!success)
    {
    	// Stop cannot be preempted
      	if (!ros::ok()) {
      		
        	success = false;
        	break;
      	}
		
		success = arm.isActionDone();
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "stopping in progress";
    	as.publishFeedback(feedback);
		
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	ROS_INFO_STREAM(arm.getName()  << ": stop succeeded");
      	result.pose = arm.getPoseCurrent().toRosToolPose();
		result.info = "stop succeeded";
      	
      	as.setSucceeded(result);
	}
	
}
 
 
void GestureServer::grasp(double angle, double speed)
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
      			<< ": grasp not possible, goal > current");
		result.info = arm.getName()  
				+ ": grasp not possible, goal > current";
      	as.setAborted(result);
      	return;
    } 
	
	// Start action
	ROS_INFO_STREAM(arm.getName() << " grasping with " << 
						angle << " degrees");
  	feedback.pose = arm.getPoseCurrent().toRosToolPose();
	feedback.info = "starting grasp";
    as.publishFeedback(feedback);
    
  	arm.moveGripper(angle, speed);
  	
  	// Wait for action to be done
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok()) {
      		
        	ROS_INFO_STREAM(arm.getName()  << ": grasp: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
		
		success = arm.isActionDone();
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "grasp in progress";
    	as.publishFeedback(feedback);
		
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	ROS_INFO_STREAM(arm.getName()  << ": grasp succeeded");
      	result.pose = arm.getPoseCurrent().toRosToolPose();
		result.info = "grasp succeeded";
      	
      	as.setSucceeded(result);
	}
	
}


void GestureServer::cut(double angle, double speed)
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
      			<< ": cut not possible, goal > current");
		result.info = arm.getName()  
				+ ": cut not possible, goal > current";
      	as.setAborted(result);
      	return;
    } 
	
	// Start action
	ROS_INFO_STREAM(arm.getName() << " grasping with " << 
						angle << " degrees");
  	feedback.pose = arm.getPoseCurrent().toRosToolPose();
	feedback.info = "starting cut";
    as.publishFeedback(feedback);
    
  	arm.moveGripper(angle, speed);
  	
  	// Wait for action to be done
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok()) {
      		
        	ROS_INFO_STREAM(arm.getName()  << ": cut: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
		
		success = arm.isActionDone();
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "cut in progress";
    	as.publishFeedback(feedback);
		
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	ROS_INFO_STREAM(arm.getName()  << ": cut succeeded");
      	result.pose = arm.getPoseCurrent().toRosToolPose();
		result.info = "cut succeeded";
      	
      	as.setSucceeded(result);
	}
	
}


void GestureServer::release(double angle, double speed)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureFeedback feedback;
    irob_msgs::GestureResult result;

	Pose p = arm.getPoseCurrent();
	double curr_angle = radToDeg(p.jaw);
	
	// Check if closing is possible
	if (curr_angle > (angle + 0.1)) {
		result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  
      			<< ": release not possible, goal > current");
		result.info = arm.getName()  
				+ ": release not possible, goal > current";
      	as.setAborted(result);
      	return;
    } 
	
	// Start action
	ROS_INFO_STREAM(arm.getName() << " relese with " << 
						angle << " degrees");
  	feedback.pose = arm.getPoseCurrent().toRosToolPose();
	feedback.info = "starting release";
    as.publishFeedback(feedback);
    
  	arm.moveGripper(angle, speed);
  	
  	// Wait for action to be done
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok()) {
      		
        	ROS_INFO_STREAM(arm.getName()  << ": release: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
		
		success = arm.isActionDone();
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "release in progress";
    	as.publishFeedback(feedback);
		
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	ROS_INFO_STREAM(arm.getName()  << ": release succeeded");
      	result.pose = arm.getPoseCurrent().toRosToolPose();
		result.info = "release succeeded";
      	
      	as.setSucceeded(result);
	}
	
}

void GestureServer::openJaws(double angle, double speed)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureFeedback feedback;
    irob_msgs::GestureResult result;

	Pose p = arm.getPoseCurrent();
	double curr_angle = radToDeg(p.jaw);
	
	// Check if closing is possible
	if (curr_angle > (angle + 0.1)) {
		result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  
      			<< ": open jaws not possible, goal > current");
		result.info = arm.getName()  
				+ ": open jaws not possible, goal > current";
      	as.setAborted(result);
      	return;
    } 
	
	// Start action
	ROS_INFO_STREAM(arm.getName() << " open jaws with " << 
						angle << " degrees");
  	feedback.pose = arm.getPoseCurrent().toRosToolPose();
	feedback.info = "starting open jaws";
    as.publishFeedback(feedback);
    
  	arm.moveGripper(angle, speed);
  	
  	// Wait for action to be done
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok()) {
      		
        	ROS_INFO_STREAM(arm.getName()  << ": open jaws: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
		
		success = arm.isActionDone();
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "open jaws in progress";
    	as.publishFeedback(feedback);
		
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	ROS_INFO_STREAM(arm.getName()  << ": open jaws succeeded");
      	result.pose = arm.getPoseCurrent().toRosToolPose();
		result.info = "open jaws succeeded";
      	
      	as.setSucceeded(result);
	}
	
}

void GestureServer::approach(Eigen::Vector3d movement, double speed)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureResult result;
    irob_msgs::GestureFeedback feedback;
    
    // Start action
	ROS_INFO_STREAM(arm.getName() << " approach with " << 
						movement << " mm");
  	feedback.pose = arm.getPoseCurrent().toRosToolPose();
	feedback.info = "starting approach";
    as.publishFeedback(feedback);

	arm.moveTool(arm.getPoseCurrent() + movement, speed);
	
	while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << ": approach: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "approach in progress";
    	as.publishFeedback(feedback);
		
		success = arm.isActionDone();
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  << ": approach succeeded");
		result.info = "approach succeeded";
      	as.setSucceeded(result);
    }
}

void GestureServer::leave(Eigen::Vector3d movement, double speed)
{
    // Helper variables
    bool success = false;
    ros::Rate loop_rate(50.0);

    irob_msgs::GestureResult result;
    irob_msgs::GestureFeedback feedback;
    
    // Start action
	ROS_INFO_STREAM(arm.getName() << " leave with " << 
						movement << " mm");
  	feedback.pose = arm.getPoseCurrent().toRosToolPose();
	feedback.info = "starting leave";
    as.publishFeedback(feedback);

	arm.moveTool(arm.getPoseCurrent() + movement, speed);
	
	while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << ": leave: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "leave in progress";
    	as.publishFeedback(feedback);
		
		success = arm.isActionDone();
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	result.pose = arm.getPoseCurrent().toRosToolPose();
      	ROS_INFO_STREAM(arm.getName()  << ": leave succeeded");
		result.info = "leave succeeded";
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

 	// Start action
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
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = "go to in progress";
    	as.publishFeedback(feedback);
		
		success = arm.isFollowTrajectoryDone();
		// TODO what if ABORTED?
  		loop_rate.sleep();
    }

	// Set the action state to succeeded
    if(success)
    {
    	ROS_INFO_STREAM(arm.getName()  << ": go to succeeded");
      	result.pose = arm.getPoseCurrent().toRosToolPose();
		result.info = arm.getName()  + ": go to succeeded";
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


