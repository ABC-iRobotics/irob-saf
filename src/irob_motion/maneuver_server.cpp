/*
 *  maneuver_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-18
 *  
 */

#include "irob_motion/maneuver_server.hpp"

namespace ias {


ManeuverServer::ManeuverServer(ros::NodeHandle nh, 
	std::vector<std::string> arm_names): 
			nh(nh),
			as(nh, "maneuver", boost::bind(
				&ManeuverServer::maneuverActionCB, this, _1), false)
{
	
	for(std::vector<std::string>::size_type i = 0; i != arm_names.size(); i++) {
   		arms.push_back(new GestureClient(nh, arm_names[i]));
	}
	
	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    as.start();
}

ManeuverServer::~ManeuverServer()
{
	for(std::vector<GestureClient*>::size_type i = 0; i != arms.size(); i++) {
   		delete(arms[i]);
	}
}

 
void ManeuverServer::subscribeTopics() 
{

}

void ManeuverServer::advertiseTopics() 
{

}


int ManeuverServer::findArmIdx(std::string arm_name)
{
	int idx = -1;
	for(std::vector<GestureClient*>::size_type i = 0; i != arms.size(); i++) {
		if (arm_name.compare(arms[i]->getName()) == 0) {
			idx = i;
			break;
		}
	}
	return idx;
}

/*
 * Callbacks
 */
void ManeuverServer::maneuverActionCB(
		const irob_autosurg::ManeuverGoalConstPtr &goal)
{
    switch(goal -> action)
    {
    	case irob_autosurg::ManeuverGoal::DISSECT:
    	{
    		std::vector<Pose> waypoints;
			for (geometry_msgs::Pose p : goal->waypoints)
				waypoints.push_back(Pose(p, 0.0));
			Pose target(goal -> target, 0.0);	
    		dissect(goal -> arm_name, target, goal -> depth,
    			goal -> closed_angle, goal -> open_angle,
    			waypoints);
    		break;
    	}	
    		
    	case irob_autosurg::ManeuverGoal::GRASP:
    	{
    		std::vector<Pose> waypoints;
			for (geometry_msgs::Pose p : goal->waypoints)
				waypoints.push_back(Pose(p, 0.0));
			Pose target(goal -> target, 0.0);	
    		grasp(goal -> arm_name, target, goal -> approach_dist,
    			goal -> closed_angle, goal -> open_angle,
    			waypoints);
    		break;
    	}	
    		
    	case irob_autosurg::ManeuverGoal::MOVE_TO:
    	{
    		std::vector<Pose> waypoints;
			for (geometry_msgs::Pose p : goal->waypoints)
				waypoints.push_back(Pose(p, 0.0));
    		Pose target(goal -> target, 0.0);	
    		moveTo(goal -> arm_name, target, waypoints);
    		break;
    	}		
    		
    	default:
    	{
    		int arm_idx = findArmIdx(goal->arm_name);
    		if (arm_idx < 0)
    			throw std::runtime_error(
					"Arm with the given name not found");
					
    		ROS_ERROR_STREAM(arms[arm_idx] -> getName()  << 
    					": invalid maneuver action code received");
    		irob_autosurg::ManeuverResult result;
    		
    		result.pose = arms[arm_idx] -> getPoseCurrent().toRosToolPose();
			result.info = arms[arm_idx] -> getName()  + 
						": invalid maneuver action code";
      		as.setAborted(result);
    		break;
    	}    	
    }	
    
}
 
 
void ManeuverServer::dissect(std::string arm_name, Pose target, double depth,
    			double closed_angle, double open_angle,
    			std::vector<Pose> waypoints)
{
    bool success = true;
    bool preempted = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::ManeuverFeedback feedback;
    irob_autosurg::ManeuverResult result;
    
    int arm_idx = findArmIdx(arm_name);
    if (arm_idx < 0)
    	throw std::runtime_error(
			"Arm with the given name not found");
    
    ROS_INFO_STREAM("Start dissection");
    
    // Close tool
    arms[arm_idx]->toolClose(closed_angle);
    while(!arms[arm_idx]->isGestureDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		
  		// TODO send feedback
    }
    
    // Go to pos
    // TODO speed not used
    
    target.jaw = degToRad(closed_angle);
    for (Pose p : waypoints)
    	p.jaw = closed_angle;
    arms[arm_idx]->goTo(target, 20.0, waypoints);
    while(!arms[arm_idx]->isGestureDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
			ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		// TODO send feedback
    }
    
    // penetrate
    arms[arm_idx]->inTCPforward(depth);
    while(!arms[arm_idx]->isGestureDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		// TODO send feedback
    }
    
    // Open tool
    arms[arm_idx]->toolOpen(open_angle);
    while(!arms[arm_idx]->isGestureDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}

  		loop_rate.sleep();
  		// TODO send feedback
    }
    
    // Pull tool out 
    arms[arm_idx]->inTCPbackward(depth);
    while(!arms[arm_idx]->isGestureDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		// TODO send feedback
    }


    if(success)
    {
      	ROS_INFO_STREAM("Dissection succeeded");
      	result.pose = arms[arm_idx]->getPoseCurrent().toRosToolPose();
		result.info = "Dissection succeeded";
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
    
}
  
void ManeuverServer::grasp(std::string arm_name, Pose target, 
				double approach_dist,
    			double closed_angle, double open_angle,
    			std::vector<Pose> waypoints)
{
    bool success = true;
    bool preempted = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::ManeuverFeedback feedback;
    irob_autosurg::ManeuverResult result;
    
    int arm_idx = findArmIdx(arm_name);
    if (arm_idx < 0)
    	throw std::runtime_error(
			"Arm with the given name not found");
    
    ROS_INFO_STREAM("Start grasping");
    
    // Open tool
    arms[arm_idx]->toolOpen(open_angle);
    while(!arms[arm_idx]->isGestureDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		
  		// TODO send feedback
    }
    
    // Calc approach params
    target.jaw = degToRad(open_angle);
    for (Pose p : waypoints)
    	p.jaw = open_angle;
    
    Eigen::Matrix3d R = target.orientation.toRotationMatrix();
    Eigen::Vector3d v(0.0, 0.0, -std::abs(approach_dist));
	v = R*v;
	
	Pose approach_pos(target + v);
    
    // Go to pos    
    
    arms[arm_idx]->goTo(approach_pos, 20.0, waypoints);
    while(!arms[arm_idx]->isGestureDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
			ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		// TODO send feedback
    }
    
    // Approach
    arms[arm_idx]->inTCPforward(approach_dist);
    while(!arms[arm_idx]->isGestureDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		// TODO send feedback
    }
    
    // Close tool
    arms[arm_idx]->toolClose(closed_angle);
    while(!arms[arm_idx]->isGestureDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		// TODO send feedback
    }

    if(success)
    {
      	ROS_INFO_STREAM("Grasp succeeded");
		result.info = "Grasp succeeded";
		result.pose = arms[arm_idx]->getPoseCurrent().toRosToolPose();
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
}

void ManeuverServer::moveTo(std::string arm_name, Pose target, std::vector<Pose> waypoints)
{
    bool success = true;
    bool preempted = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::ManeuverFeedback feedback;
    irob_autosurg::ManeuverResult result;
    
    int arm_idx = findArmIdx(arm_name);
    if (arm_idx < 0)
    	throw std::runtime_error(
			"Arm with the given name not found");
    
    ROS_INFO_STREAM("Start moving to target");
    
    
    // Go to pos   
    double jaw_curr = arms[arm_idx] -> getPoseCurrent().jaw;
    target.jaw = jaw_curr;
    for (Pose p : waypoints)
    	p.jaw = jaw_curr;
    arms[arm_idx]->goTo(target, 20.0, waypoints);
    while(!arms[arm_idx]->isGestureDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
			ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}
		
		// TODO this can be determined by the details received 
		// in the arm's doneCB
  		loop_rate.sleep();
  		// TODO send feedback
    }
    
  
    if(success)
    {
      	ROS_INFO_STREAM("Grasp succeeded");
		result.info = "Grasp succeeded";
		result.pose = arms[arm_idx]->getPoseCurrent().toRosToolPose();
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
}
  
 

}

using namespace ias;

/**
 * Maneuver server main 
 */
int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "maneuver_server");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

	std::vector<std::string> arm_names;
	priv_nh.getParam("arm_names", arm_names);
	
	
    
    // StartGesture server
  	try {
    	ManeuverServer ms(nh, arm_names);
    	
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


