/*
 * 	maneuver_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-31
 *  
 */

#include "irob_motion/maneuver_client.hpp"

namespace ias {


ManeuverClient::ManeuverClient(ros::NodeHandle nh, 
					std::vector<std::string> arm_names): 
			nh(nh), arm_names(arm_names),
			ac("maneuver", true)
{

	// Subscribe and advertise topics
	subscribeTopics();
    waitForActionServer();
}

ManeuverClient::~ManeuverClient()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
 
// Read pos 
void ManeuverClient::positionCartesianCurrentCB(
				const irob_autosurg::ToolPoseStampedConstPtr& msg,
				const std::string& arm_name) 
{
    position_cartesian_current[arm_name] = *msg;
}

void ManeuverClient::subscribeTopics() 
{      
	for(std::vector<std::string>::size_type i = 0; i != arm_names.size(); i++) {
   		position_cartesian_current_subs[arm_names[i]] = 
   			nh.subscribe<irob_autosurg::ToolPoseStamped>(
                        "maneuver/"+arm_names[i]
                        +"/position_cartesian_current_cf",
                       	1000, 
                       	boost::bind(
						&ManeuverClient::positionCartesianCurrentCB, 
						this, _1, arm_names[i]));
	}
}


void ManeuverClient::waitForActionServer() 
{
	ROS_INFO_STREAM("Wating for action servers...");
	ac.waitForServer();
    ROS_INFO_STREAM("Action servers started");
}



// Robot motions

void ManeuverClient::dissect(std::string arm_name, 
								Pose pos, 
								double depth,
								double closed_angle,
								double open_angle,
								std::vector<Pose> waypoints 
								/* = std::vector<Pose>()*/)	
{
    // Send a goal to the action
  	irob_autosurg::ManeuverGoal goal;
  	
  	goal.action = irob_autosurg::ManeuverGoal::DISSECT;
 
  	goal.arm_name = arm_name;
  	goal.target = pos.toRosPose();
  	
  	for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
    	
  	goal.depth = depth;
  	goal.closed_angle = closed_angle;
  	goal.open_angle = open_angle;
  	
  	ac.sendGoal(goal);
}

void ManeuverClient::grasp(std::string arm_name, 
								Pose pos, double open_angle,
								double closed_angle,
								double approach_dist,
								std::vector<Pose> waypoints 
								/* = std::vector<Pose>()*/)
{
    // Send a goal to the action
  	irob_autosurg::ManeuverGoal goal;
  	
  	goal.action = irob_autosurg::ManeuverGoal::GRASP;
 
  	goal.arm_name = arm_name;
  	goal.target = pos.toRosPose();
  	
  	for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
  	
  	goal.closed_angle = closed_angle;
  	goal.open_angle = open_angle;
  	goal.approach_dist = approach_dist;
  	
  	ac.sendGoal(goal);
}

void ManeuverClient::moveTo(std::string arm_name, 
								Pose pos,
								std::vector<Pose> waypoints 
								/* = std::vector<Pose>()*/)
{
    // Send a goal to the action
  	irob_autosurg::ManeuverGoal goal;
  	
  	goal.action = irob_autosurg::ManeuverGoal::DISSECT;
 
  	goal.arm_name = arm_name;
  	goal.target = pos.toRosPose();
  	
  	for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
  	
  	ac.sendGoal(goal);
}


bool ManeuverClient::isManeuverDone()
{
	return ac.isDone();
}

Pose ManeuverClient::getPoseCurrent(std::string arm_name)
{
 	while (position_cartesian_current[arm_name].header.seq == 0)
	{
 		ros::spinOnce();
 		ros::Duration(0.05).sleep();
 	}
 	Pose ret(position_cartesian_current[arm_name]);
 	return ret;

}



}


























