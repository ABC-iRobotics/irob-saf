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

void GestureClient::closeToolDoneCB(
			const actionlib::SimpleClientGoalState& state,
            const irob_autosurg::CloseToolResultConstPtr& result)
{
	close_tool_done = true;
}

void GestureClient::openToolDoneCB(
			const actionlib::SimpleClientGoalState& state,
            const irob_autosurg::OpenToolResultConstPtr& result)
{
	open_tool_done = true;
}


void GestureClient::penetrateDoneCB(
			const actionlib::SimpleClientGoalState& state,
            const irob_autosurg::PenetrateResultConstPtr& result)
{
	penetrate_done = true;
}


void GestureClient::goToDoneCB(
			const actionlib::SimpleClientGoalState& state,
            const irob_autosurg::GoToResultConstPtr& result)
{
	go_to_done = true;
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
  	
  	close_tool_done = false;
  	
  	close_tool_ac.sendGoal(goal, boost::bind(				
  						&GestureClient::closeToolDoneCB, this, _1, _2));
}

void GestureClient::openTool(double angle, double speed /* = 10.0 */)
{
    // Send a goal to the action
  	irob_autosurg::OpenToolGoal goal;
 
  	goal.angle = angle;
  	goal.speed = speed;
  	
  	open_tool_done = false;
  	
  	open_tool_ac.sendGoal(goal, boost::bind(				
  						&GestureClient::openToolDoneCB, this, _1, _2));
}


void GestureClient::penetrate(double depth, double speed /*=10.0*/)
{
	irob_autosurg::PenetrateGoal goal;
 
  	goal.depth = depth;
  	goal.speed = speed;
   	
   	penetrate_done = false;
   	
  	penetrate_ac.sendGoal(goal, boost::bind(				
  						&GestureClient::penetrateDoneCB, this, _1, _2));
  	
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

   	
   	go_to_done = false;
   	
  	go_to_ac.sendGoal(goal, boost::bind(				
  						&GestureClient::goToDoneCB, this, _1, _2));
  	
  	// Not waiting for action finish here, a notification will be received
  	// in goToDoneCB
}


bool GestureClient::isCloseToolDone()
{
	ros::spinOnce();
	return close_tool_done;
}

bool GestureClient::isOpenToolDone()
{
	ros::spinOnce();
	return open_tool_done;
}

bool GestureClient::isPenetrateDone()
{
	ros::spinOnce();
	return penetrate_done;
}

bool GestureClient::isGoToDone()
{
	ros::spinOnce();
	return go_to_done;
}





























