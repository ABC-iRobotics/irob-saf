/*
 *  maneuver_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-18
 *  
 */

#include "irob_motion/maneuver_server.hpp"

using namespace ias;


ManeuverServer::ManeuverServer(ros::NodeHandle nh, 
	std::vector<std::string> arm_names): 
			nh(nh),
			dissect_as(nh, "maneuver/dissect", boost::bind(
				&ManeuverServer::dissectActionCB, this, _1), false),
			grasp_as(nh, "maneuver/grasp", boost::bind(
				&ManeuverServer::graspActionCB, this, _1), false)
{
	
	for(std::vector<std::string>::size_type i = 0; i != arm_names.size(); i++) {
   		arms.push_back(new GestureClient(nh, arm_names[i]));
	}
	
	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    startActionServers();
}

ManeuverServer::~ManeuverServer()
{
	for(std::vector<GestureClient*>::size_type i = 0; i != arms.size(); i++) {
   		delete(arms[i]);
	}
}

/*
 * Callbacks
 */
void ManeuverServer::dissectActionCB(
		const irob_autosurg::DissectGoalConstPtr &goal)
{
    bool success = true;
    bool preempted = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::DissectFeedback feedback;
    irob_autosurg::DissectResult result;
    
    int arm_idx = findArmIdx(goal -> arm_name);
    if (arm_idx < 0)
    	throw std::runtime_error(
			"Arm with the given name not found");
    
    ROS_INFO_STREAM("Start dissection");
    
    // Close tool
    arms[arm_idx]->closeTool(goal->closed_angle);
    while(!arms[arm_idx]->isCloseToolDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (dissect_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	dissect_as.setPreempted();
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
    
    std::vector<Pose> waypoints;
	for (geometry_msgs::Pose p : goal->waypoints)
		waypoints.push_back(Pose(p,  degToRad(goal->closed_angle)));
    
    arms[arm_idx]->goTo(Pose(goal->target, degToRad(goal->closed_angle)), 20.0, waypoints);
    while(!arms[arm_idx]->isGoToDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (dissect_as.isPreemptRequested() || !ros::ok())
      	{
			ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	dissect_as.setPreempted();
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
    arms[arm_idx]->pushIn(goal->depth);
    while(!arms[arm_idx]->isPushInDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (dissect_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	dissect_as.setPreempted();
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
    arms[arm_idx]->openTool(goal->open_angle);
    while(!arms[arm_idx]->isOpenToolDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (dissect_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	dissect_as.setPreempted();
        	success = false;
        	preempted = true;
        	break;
      	}

  		loop_rate.sleep();
  		// TODO send feedback
    }
    
    // Pull tool out 
    arms[arm_idx]->pullOut(goal->depth);
    while(!arms[arm_idx]->isPullOutDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (dissect_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	dissect_as.setPreempted();
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
		result.info = "Dissection succeeded";
      	// set the action state to succeeded
      	dissect_as.setSucceeded(result);
    }
    
}
  
void ManeuverServer::graspActionCB(
		const irob_autosurg::GraspGoalConstPtr &goal)
{
    bool success = true;
    bool preempted = false;
    ros::Rate loop_rate(50.0);

    irob_autosurg::GraspFeedback feedback;
    irob_autosurg::GraspResult result;
    
    int arm_idx = findArmIdx(goal -> arm_name);
    if (arm_idx < 0)
    	throw std::runtime_error(
			"Arm with the given name not found");
    
    ROS_INFO_STREAM("Start grasping");
    
    // Open tool
    arms[arm_idx]->openTool(goal->open_angle);
    while(!arms[arm_idx]->isOpenToolDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (grasp_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	grasp_as.setPreempted();
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
    Pose tgt(goal->target, degToRad(goal->open_angle));
    Eigen::Matrix3d R = tgt.orientation.toRotationMatrix();
    Eigen::Vector3d v(0.0, 0.0, -std::abs(goal->approach_dist));
	v = R*v;
	
	Pose approach_pos(tgt + v);
    
    // Go to pos    
    std::vector<Pose> waypoints;
	for (geometry_msgs::Pose p : goal->waypoints)
		waypoints.push_back(Pose(p,  degToRad(goal->open_angle)));
    
    arms[arm_idx]->goTo(approach_pos, 20.0, waypoints);
    while(!arms[arm_idx]->isGoToDone() && !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (grasp_as.isPreemptRequested() || !ros::ok())
      	{
			ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	grasp_as.setPreempted();
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
    arms[arm_idx]->pushIn(goal->approach_dist);
    while(!arms[arm_idx]->isPushInDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (grasp_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	grasp_as.setPreempted();
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
    arms[arm_idx]->closeTool(goal->closed_angle);
    while(!arms[arm_idx]->isCloseToolDone()&& !preempted)
    {
    	// Check that preempt has not been requested by the client
      	if (grasp_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Dissect: Preempted");
        	// Set the action state to preempted
        	grasp_as.setPreempted();
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
      	// set the action state to succeeded
      	grasp_as.setSucceeded(result);
    }
}
  
  
void ManeuverServer::subscribeTopics() 
{

}

void ManeuverServer::advertiseTopics() 
{

}

void ManeuverServer::startActionServers() 
{
	dissect_as.start();
	grasp_as.start();
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


