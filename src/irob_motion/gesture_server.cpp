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
				&GestureServer::penetrateActionCB, this, _1), false)
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

    irob_autosurg::CloseToolFeedback feedback;
    irob_autosurg::CloseToolResult result;

	ROS_INFO_STREAM(arm_typ.name << " closing tool");
  	
  	arm.moveGripper(goal->angle, goal->speed);
  	
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (close_tool_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm_typ.name << " close tool: Preempted");
        	// Set the action state to preempted
        	close_tool_as.setPreempted();
        	success = false;
        	break;
      	}
		
		success = close_tool_as.waitForResult(ros::Duration(0.5));
  		
  		// TODO send feedback
    }

    if(success)
    {
      	result.descript = robot_state.data;
      	ROS_INFO_STREAM(arm_typ.name << " initilaization succeeded");
		result.info = arm_typ.name + " initilaization succeeded";
      	// set the action state to succeeded
      	init_as.setSucceeded(result);
    }
}
  
void GestureServer::releaseActionCB(const irob_autosurg::ResetPoseGoalConstPtr &goal)
{
    // helper variables
    bool success = false;
    
    irob_autosurg::ResetPoseFeedback feedback;
    irob_autosurg::ResetPoseResult result;

	ROS_INFO_STREAM("Starting " << arm_typ.name << " pose reset");
  
    	// Check that preempt has not been requested by the client
    if (reset_pose_as.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO_STREAM(arm_typ.name << " pose reset: Preempted");
        // Set the action state to preempted
        reset_pose_as.setPreempted();
        success = false;
    }
  	
  	ROS_INFO_STREAM(arm_typ.name << " pose reset not implemented");
  	success = true;	
  	// Send some feedback
  	feedback.status = "done";
    reset_pose_as.publishFeedback(feedback);

    if(success)
    {
      result.descript = "done";
      ROS_INFO_STREAM(arm_typ.name << " pose reset succeded");
      // set the action state to succeeded
      reset_pose_as.setSucceeded(result);
    }
}
  
  
void GestureServer::followTrajectoryActionCB(
		const irob_autosurg::FollowTrajectoryGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;
    irob_autosurg::FollowTrajectoryFeedback feedback;
    irob_autosurg::FollowTrajectoryResult result;

	ROS_INFO_STREAM("Starting trajectory follow action.");
	
	Trajectory<Pose> tr(goal->trajectory);
	// TODO hande-eye calibration
	
	ros::Rate loop_rate(1.0/tr.dt);
	// start executing the action
	for (int i = 0; i < tr.size(); i++)
	{
		// check that preempt has not been requested by the client
      	if (follow_tr_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Follow trajectory: Preempted");
       	 	// set the action state to preempted
        	follow_tr_as.setPreempted();
        	success = false;
        	break;
      	}
		moveCartesianAbsolute(tr[i],tr.dt);
		
		feedback.pose = tr[i].toRosToolPose();
      	follow_tr_as.publishFeedback(feedback);
		
		loop_rate.sleep();
	}
	
    if(success)
    {
      result.pose = getPoseCurrent().toRosToolPose();
      ROS_INFO_STREAM("Follow trajectory: Succeeded");
      // set the action state to succeeded
      follow_tr_as.setSucceeded(result);
    }
  }


void GestureServer::positionCartesianCurrentCB(
				const geometry_msgs::PoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
	irob_autosurg::ToolPoseStamped fwd;
	fwd.header = position_cartesian_current.header;

 	Pose tmp(position_cartesian_current, 0.0);

	// TODO hand-eye calibration
	
	fwd.pose = tmp.toRosToolPose();
    position_cartesian_current_pub.publish(fwd);
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



