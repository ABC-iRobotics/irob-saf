/*
 * 	gesture_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-24
 *  
 */

#include "irob_motion/gesture_client.hpp"

using namespace ias;


GestureClient::GestureClient(ros::NodeHandle nh, std::string arm_name): 
			nh(nh), arm_name(arm_name),
			close_tool_ac("gesture/"+arm_name+"/close_tool", true),
			open_tool_ac("gesture/"+arm_name+"/open_tool", true),
			penetrate_ac("gesture/"+arm_name+"/penetrate", true),
			go_to_ac("gesture/"+arm_name+"/go_to", true)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    waitForActionServers();
}

GestureClient::~GestureClient()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */

// Read pos 
void GestureClient::positionCartesianCurrentCB(
				const irob_autosurg::ToolPoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
    position_cartesian_current_pub.publish(msg);
}


// TODO remap
void GestureClient::subscribeTopics() 
{                 	            	
   	position_cartesian_current_sub = 
   			nh.subscribe<irob_autosurg::ToolPoseStamped>(
                        "gesture/"+arm_name+"/position_cartesian_current_cf",
                       	1000, &GestureClient::positionCartesianCurrentCB,this);
}

// TODO remap

void GestureClient::advertiseTopics() 
{
	position_cartesian_current_pub 
				= nh.advertise<irob_autosurg::ToolPoseStamped>(
                    	"maneuver/"+arm_name+"/position_cartesian_current_cf",
                        1000);   
}


void GestureClient::waitForActionServers() 
{
	ROS_INFO_STREAM("Wating for action servers...");
	close_tool_ac.waitForServer();
	open_tool_ac.waitForServer();
    penetrate_ac.waitForServer();
    go_to_ac.waitForServer();
    ROS_INFO_STREAM("Action servers started");
}


Pose GestureClient::getPoseCurrent()
{
 	while (position_cartesian_current.header.seq == 0)
	{
 		ros::spinOnce();
 		ros::Duration(0.05).sleep();
 	}
 	Pose ret(position_cartesian_current);
 	return ret;

}

std::string GestureClient::getName()
{
	return arm_name;
}
   	

// Robot motions

void GestureClient::closeTool(double angle, double speed /* = 10.0 */)
{
    // Send a goal to the action
  	irob_autosurg::CloseToolGoal goal;
 
  	goal.angle = angle;
  	goal.speed = speed;
  	
  	close_tool_ac.sendGoal(goal);
}

void GestureClient::openTool(double angle, double speed /* = 10.0 */)
{
    // Send a goal to the action
  	irob_autosurg::OpenToolGoal goal;
 
  	goal.angle = angle;
  	goal.speed = speed;
  	
  	open_tool_ac.sendGoal(goal);
}


void GestureClient::penetrate(double depth, double speed /*=0.01*/)
{
	irob_autosurg::PenetrateGoal goal;
 
  	goal.depth = depth;
  	goal.speed = speed;
   	
  	penetrate_ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in followTrajectoryDoneCB
}

void GestureClient::goTo(Pose target, double speed /* = 0.01 */,
			std::vector<Pose> waypoints /* = empty vector */, 
			InterpolationMethod interp_method /* = LINEAR */)
{
	irob_autosurg::GoToGoal goal;
    goal.target = target.toRosToolPose();
    goal.speed = speed;
    for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosToolPose());
    
    if (interp_method == InterpolationMethod::LINEAR)
    	goal.interpolation = irob_autosurg::GoToGoal::LINEAR;
    else
    	goal.interpolation = irob_autosurg::GoToGoal::BEZIER;
    	
  	go_to_ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in goToDoneCB
}


bool GestureClient::isCloseToolDone()
{
	return close_tool_ac.isDone();
}

bool GestureClient::isOpenToolDone()
{
	return open_tool_ac.isDone();
}

bool GestureClient::isPenetrateDone()
{
	return penetrate_ac.isDone();
}

bool GestureClient::isGoToDone()
{
	return go_to_ac.isDone();
}





























