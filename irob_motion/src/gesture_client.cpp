/*
 * 	gesture_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-24
 *  
 */

#include <irob_motion/gesture_client.hpp>

namespace ias {


GestureClient::GestureClient(ros::NodeHandle nh, std::string arm_name): 
			nh(nh), arm_name(arm_name),
			ac("gesture/"+arm_name, true)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    waitForActionServer();
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
				const irob_msgs::ToolPoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
    position_cartesian_current_pub.publish(msg);
}


void GestureClient::subscribeTopics() 
{                 	            	
   	position_cartesian_current_sub = 
   			nh.subscribe<irob_msgs::ToolPoseStamped>(
                        "gesture/"+arm_name+"/position_cartesian_current_cf",
                       	1000, &GestureClient::positionCartesianCurrentCB,this);
}


void GestureClient::advertiseTopics() 
{
	position_cartesian_current_pub 
				= nh.advertise<irob_msgs::ToolPoseStamped>(
                    	"maneuver/"+arm_name+"/position_cartesian_current_cf",
                        1000);   
}


void GestureClient::waitForActionServer() 
{
	ROS_INFO_STREAM("Wating for action server...");
	ac.waitForServer();
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

void GestureClient::toolClose(double angle, double speed /* = 10.0 */)
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::TOOL_CLOSE;
 	
  	goal.angle = angle;
  	goal.speed = speed;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}

void GestureClient::toolOpen(double angle, double speed /* = 10.0 */)
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::TOOL_OPEN;
 
  	goal.angle = angle;
  	goal.speed = speed;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


void GestureClient::inTCPforward(double distance, double speed /*=10.0*/)
{
	// Send a goal to the action
	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::IN_TCP_FORWARD;
 
  	goal.distance = distance;
  	goal.speed = speed;
   	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}

void GestureClient::inTCPbackward(double distance, double speed /*=10.0*/)
{
	// Send a goal to the action
	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::IN_TCP_BACKWARD;
 
  	goal.distance = distance;
  	goal.speed = speed;
   	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}

void GestureClient::goTo(Pose target, double speed /* = 10.0 */,
			std::vector<Pose> waypoints /* = empty vector */, 
			InterpolationMethod interp_method /* = LINEAR */)
{
	// Send a goal to the action
	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::GO_TO;
 	
    goal.target = target.toRosToolPose();
    goal.speed = speed;
    for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosToolPose());
    
    if (interp_method == InterpolationMethod::LINEAR)
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_LINEAR;
    else
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_BEZIER;
    	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


bool GestureClient::isGestureDone()
{
	return ac.isDone();
}

}






















