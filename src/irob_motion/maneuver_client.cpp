/*
 * 	maneuver_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-31
 *  
 */

#include "irob_motion/maneuver_client.hpp"

using namespace ias;


ManeuverClient::ManeuverClient(ros::NodeHandle nh): 
			nh(nh),
			dissect_ac("dissect", true),
			grasp_ac("grasp", true)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    waitForActionServers();
}

ManeuverClient::~ManeuverClient()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */


void ManeuverClient::waitForActionServers() 
{
	ROS_INFO_STREAM("Wating for action servers...");
	dissect_ac.waitForServer();
	grasp_ac.waitForServer();
    ROS_INFO_STREAM("Action servers started");
}



// Robot motions

void ManeuverClient::dissect(std::string arm_name, 
								Pose pos, double depth,
								double closed_angle, double open_angle)	
{
    // Send a goal to the action
  	irob_autosurg::DissectGoal goal;
 
  	goal.arm_name = arm_name;
  	goal.pose = pos.toRosPose();
  	goal.depth = depth;
  	goal.closed_angle = closed_angle;
  	goal.open_angle = open_angle;
  	
  	dissect_ac.sendGoal(goal);
}

void ManeuverClient::grasp(std::string arm_name, 
								Pose pos, double open_angle,
								double closed_angle)
{
    // Send a goal to the action
  	irob_autosurg::GraspGoal goal;
 
  	goal.arm_name = arm_name;
  	goal.pose = pos.toRosPose();
  	goal.closed_angle = closed_angle;
  	goal.open_angle = open_angle;
  	
  	grasp_ac.sendGoal(goal);
}


bool ManeuverClient::isDissectDone()
{
	return dissect_ac.isDone();
}

bool ManeuverClient::isGraspDone()
{
	return grasp_ac.isDone();
}






























