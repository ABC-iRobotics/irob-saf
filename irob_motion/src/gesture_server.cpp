/*
 *  gesture_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-06
 *  
 */

#include <irob_motion/gesture_server.hpp>

namespace ias {


const double GestureServer::DEFAULT_LOOP_RATE = 50.0;				// Hz


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
	// Extract data from msg
	InterpolationMethod	interp_method;
	if (goal -> interpolation == goal -> INTERPOLATION_BEZIER)
		interp_method = InterpolationMethod::BEZIER;
	else
		interp_method = InterpolationMethod::LINEAR;
	
	Pose target(goal -> target, 0.0);
	Pose approach_pose(goal -> approach_pose, 0.0);
	std::vector<Pose> waypoints;
			for (geometry_msgs::Pose p : goal->waypoints)
				waypoints.push_back(Pose(p, 0.0));
				
	Eigen::Vector3d displacement(goal->displacement.x,
										goal->displacement.y,
										goal->displacement.z);
										
	try {									
		// Do the action			
    	switch(goal -> action)
    	{
    		// STOP
    		case irob_msgs::GestureGoal::STOP:
    		{
    			stop();
    			break;
    		}	
    	
    		// STANDBY
    		case irob_msgs::GestureGoal::NAV_TO_POS:
    		{
    			nav_to_pos(target, waypoints,
    					interp_method, goal -> speed_cartesian);
    			break;
    		}
    	
    		// GRASP
    		case irob_msgs::GestureGoal::GRASP:
    		{				
  		  		grasp(target, approach_pose,
						goal -> open_angle, goal -> closed_angle,
						waypoints, interp_method,
						goal -> speed_cartesian, goal -> speed_jaw);
	    		break;
	    	}
    	
	    	// CUT
	    	case irob_msgs::GestureGoal::CUT:
	    	{
	    		cut(target, approach_pose,
						goal -> open_angle, goal -> closed_angle,
						waypoints, interp_method,
						goal -> speed_cartesian, goal -> speed_jaw);
	    		break;
	    	}
	    	
	    	// PUSH
	    	case irob_msgs::GestureGoal::PUSH:
	    	{
	    		push(target, approach_pose, 
					displacement, goal -> closed_angle,
					waypoints, interp_method,
					goal -> speed_cartesian, goal -> speed_jaw);
	    		break;
	    	}
	    	
	    	// DISSECT
	    	case irob_msgs::GestureGoal::DISSECT:
		    {
	    		dissect(target, approach_pose, 
					displacement, goal -> open_angle, goal -> closed_angle,
					waypoints, interp_method,
					goal -> speed_cartesian, goal -> speed_jaw);
    			break;
    		}
    		
    		// PLACE
    		case irob_msgs::GestureGoal::PLACE:
    		{
    			place( target, approach_pose,
					waypoints, interp_method, goal -> speed_cartesian);
    			break;
    		}
    		
    		// MANIPULATE
    		case irob_msgs::GestureGoal::MANIPULATE:
    		{
    			manipulate(displacement, goal -> speed_cartesian);
    			break;
    		}
    		
    		// RELEASE
    		case irob_msgs::GestureGoal::RELEASE:
    		{
    			release(approach_pose, goal -> open_angle,
						goal -> speed_cartesian, goal -> speed_jaw);
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
    } catch (std::runtime_error e) {
    	std::string str_abort = "abort";
  		std::string str_err = e.what();
  		if (str_err.find(str_abort) != std::string::npos)
    		ROS_ERROR_STREAM(e.what());
    	else
    		throw e;
    }
}



// Helper methods
 
/**
 *	Wait for actionDone()
 */
bool GestureServer::waitForActionDone(std::string stage)
{
	bool done = false;
    ros::Rate loop_rate(DEFAULT_LOOP_RATE);

    irob_msgs::GestureFeedback feedback;
    
	while(!done)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm.getName()  << ": " << stage << ": Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	break;
      	}
		
		// Send some feedback
		feedback.pose = arm.getPoseCurrent().toRosToolPose();
		feedback.info = stage;
    	as.publishFeedback(feedback);
		
		done = arm.isActionDone();
		loop_rate.sleep();
    }
    
    return done;
}

/**
 *	Evaluate action state
 */
bool GestureServer::handleActionState(std::string stage, 
										bool lastStage /* = false */ )
{
	irob_msgs::GestureResult result;
    irob_msgs::GestureFeedback feedback;
    
   	switch (arm.getState().state_)	
   	{
   		case actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED:
   			ROS_INFO_STREAM(arm.getName()  << ": " << stage << " succeeded");
   			feedback.pose = arm.getPoseCurrent().toRosToolPose();
			feedback.info = stage + " succeeded";
			as.publishFeedback(feedback);
			
			// Do not send succeeded if not the last stage,
			// else the gesture continues
			if (lastStage)
			{
     			result.pose = arm.getPoseCurrent().toRosToolPose();
				result.info = stage + " succeeded";
  				as.setSucceeded(result);
  			}
  				
   			break;
   		case actionlib::SimpleClientGoalState::StateEnum::ABORTED:
      		// Send ABORTED to the upper level node
      		// and throw an error to finish gesture execution
      		ROS_ERROR_STREAM(arm.getName()  << ": " << stage << " aborted");
      		result.pose = arm.getPoseCurrent().toRosToolPose();
			result.info = stage + " aborted";
				
      		as.setAborted(result);
      			
      		throw std::runtime_error(
      				"Action aborted by node on a lower level.");
      		break;
      	default:
      		// PREEMPTED or similar, do nothing
      		break;
    }
}



/**
 * -----------------------------------------------------------------------------
 * Actions
 * -----------------------------------------------------------------------------
 */
 
 
/**
 * Stop
 */ 
void GestureServer::stop()
{
     // Helper variables
    bool done = false;
    std::string stage = "stop";

 	// Start action
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.stop();
  	
    done = waitForActionDone(stage);

	if (done)
		handleActionState(stage, true);
	else
		return;
	
}

/**
 * Nav_to_pos
 */
void GestureServer::nav_to_pos(Pose target, std::vector<Pose> waypoints,
			InterpolationMethod interp_method, double speed_cartesian)
{

	// Helper variables
    bool done = false;
    std::string stage = "navigate";

 	// Start action
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveTool(target, speed_cartesian, waypoints, interp_method);
  	
    done = waitForActionDone(stage);

	if (done)
		handleActionState(stage, true);
	else
		return;
}
 
/**
 * Grasp
 */ 
void GestureServer::grasp(Pose target, Pose approach_pose,
			double open_angle, double closed_angle,
			std::vector<Pose> waypoints, InterpolationMethod interp_method,
			double speed_cartesian, double speed_jaw)
{
   	// Helper variables   	
    bool done = false;
    std::string stage = "";

 	// Start action
 	
 	// Navigate
 	stage = "navigate";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(approach_pose, speed_cartesian, waypoints, interp_method);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
 	
 	// Open tool 
 	stage = "open_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(open_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Approach
 	stage = "approach";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(target, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Close tool 
	stage = "close_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(closed_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, true);
	else
		return;
}

/**
 * Cut
 */
void GestureServer::cut(Pose target, Pose approach_pose,
			double open_angle, double closed_angle,
			std::vector<Pose> waypoints, InterpolationMethod interp_method,
			double speed_cartesian, double speed_jaw)
{
	// Helper variables   	
    bool done = false;
    std::string stage = "";

 	// Start action
 	
 	// Navigate
 	stage = "navigate";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(approach_pose, speed_cartesian, waypoints, interp_method);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
 	
 	// Open tool 
 	stage = "open_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(open_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Approach
 	stage = "approach";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(target, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Close tool 
	stage = "close_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(closed_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, true);
	else
		return;
}

/**
 * Release
 */
void GestureServer::release(Pose approach_pose,
			double open_angle,
			double speed_cartesian, double speed_jaw)
{
	// Helper variables   	
    bool done = false;
    std::string stage = "";

 	// Start action
 	
 	// Open tool 
 	stage = "open_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(open_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Approach
 	stage = "leave";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(approach_pose, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, true);
	else
		return;
	
}

/**
 * Place
 */
void GestureServer::place(Pose target, Pose approach_pose,
			std::vector<Pose> waypoints, InterpolationMethod interp_method,
			double speed_cartesian)
{
   	// Helper variables   	
    bool done = false;
    std::string stage = "";

 	// Start action
 	
 	// Navigate
 	stage = "navigate";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(approach_pose, speed_cartesian, waypoints, interp_method);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Approach
 	stage = "approach";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(target, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, true);
	else
		return;
	
}

/**
 * Push
 */
void GestureServer::push(Pose target, Pose approach_pose, 
			Eigen::Vector3d displacement, double closed_angle,
			std::vector<Pose> waypoints, InterpolationMethod interp_method,
			double speed_cartesian, double speed_jaw)
{
    // Helper variables   	
    bool done = false;
    std::string stage = "";

 	// Start action
 	
 	// Navigate
 	stage = "navigate";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(approach_pose, speed_cartesian, waypoints, interp_method);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
 	
 	// Close tool 
 	stage = "close_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(closed_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Approach
 	stage = "approach";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(target, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Push
 	stage = "push";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	Pose pushed_pose = arm.getPoseCurrent() + displacement;
 	arm.moveTool(pushed_pose, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, true);
	else
		return;
}

/**
 * Dissect
 */
void GestureServer::dissect(Pose target, Pose approach_pose, 
			Eigen::Vector3d displacement,
			double open_angle, double closed_angle,
			std::vector<Pose> waypoints, InterpolationMethod interp_method,
			double speed_cartesian, double speed_jaw)
{
    // Helper variables   	
    bool done = false;
    std::string stage = "";

 	// Start action
 	
 	// Navigate
 	stage = "navigate";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(approach_pose, speed_cartesian, waypoints, interp_method);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
 	
 	// Close tool 
 	stage = "close_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(closed_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Approach
 	stage = "approach";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	arm.moveTool(target, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Penetrate
 	stage = "penetrate";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	Pose pushed_pose = arm.getPoseCurrent() + displacement;
 	arm.moveTool(pushed_pose, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Close tool 
 	stage = "close_tool";	
	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  	arm.moveJaws(open_angle, speed_jaw);
  	
    done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, false);
	else
		return;
		
	// Pull
 	stage = "pull";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	pushed_pose = arm.getPoseCurrent() - displacement;
 	arm.moveTool(pushed_pose, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, true);
	else
		return;		
}
  
/**
 * Maipulate
 */  
// TODO rotation?
void GestureServer::manipulate(Eigen::Vector3d displacement,
								double speed_cartesian)
{
    // Helper variables   	
    bool done = false;
    std::string stage = "";

 	// Start action
		
	// Manipulate
 	stage = "manipulate";
 	ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
 	Pose manipulated_pose = arm.getPoseCurrent() + displacement;
 	arm.moveTool(manipulated_pose, speed_cartesian);
 	
 	done = waitForActionDone(stage);
	if (done)
		handleActionState(stage, true);
	else
		return;
		
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


