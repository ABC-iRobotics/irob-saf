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
			close_tool_ac("close_tool", true),
			open_tool_ac("open_tool", true),
			penetrate_ac("penetrate", true),
			go_to_ac("go_to", true)
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
    // position_cartesian_current_pub.publish(msg);
    // TODO need forwarding??
}


// TODO remap
void GestureClient::subscribeTopics() 
{                 	            	
   	position_cartesian_current_sub = 
   			nh.subscribe<irob_autosurg::ToolPoseStamped>(
                        "position_cartesian_current_in",
                       	1000, &GestureClient::positionCartesianCurrentCB,this);
}

// TODO remap
/*
void GestureClient::advertiseTopics() 
{
	position_cartesian_current_pub 
				= nh.advertise<irob_autosurg::ToolPoseStamped>(
                    	"position_cartesian_current_out",
                        1000);   
}
*/

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
 	ros::spinOnce();
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


void GestureClient::penetrate(double depth, double speed /*=10.0*/)
{
	irob_autosurg::PenetrateGoal goal;
 
  	goal.depth = depth;
  	goal.speed = speed;
   	
  	penetrate_ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in followTrajectoryDoneCB
}

void GestureClient::goTo(Pose target, double speed /* = 10.0 */,
			std::vector<Pose> waypoints /* = empty vector */, 
			InterpolationMethod interp_method /* = LINEAR */)
{
	irob_autosurg::GoToGoal goal;
 
 	ROS_INFO_STREAM("GoTo action not implemented yet");
 	// TODO not implemented yet

   	
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





























