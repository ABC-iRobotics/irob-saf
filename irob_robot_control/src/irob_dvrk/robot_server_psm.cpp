/*
 *  robot_server_psm.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
 *  
 */

#include <irob_dvrk/robot_server_psm.hpp>
#include <numeric>
#include <chrono>
#include <irob_utils/trajectory_factory.hpp>

namespace ias {

                   
const std::string RobotServerPSM::ERROR_INSIDE_CANNULA
                    ="make sure the tool is inserted past the cannula";
                    
const double RobotServerPSM::INSERTION_DEPTH = 0.055;
const double RobotServerPSM::INSERTION_SPEED = 0.05;
const double RobotServerPSM::INSERTION_DT = 0.01;
const int RobotServerPSM::INSERTION_JOINT_IDX = 2;


RobotServerPSM::RobotServerPSM(ros::NodeHandle nh, ArmTypes arm_typ, std::string arm_name, 
	std::string regfile, bool isActive): 
		RobotServerDVRK(nh, arm_typ, arm_name, regfile, isActive)
{
	if (!(arm_typ == ArmTypes::PSM1 || 
  							arm_typ == ArmTypes::PSM2))
  		throw std::runtime_error(
  		"Tried to create RobotServerPSM object for ECM or MTM arm type.");
}

void RobotServerPSM::initArm(bool move_allowed)
{	
	typedef enum init_action_type 
    	{CARTESIAN, JOINT, INSERT} 
    	init_action_type_t;
    	
    // helper variables
    ros::Rate loop_rate(2);
    ros::Rate insert_rate(1.0/INSERTION_DT);
    bool success = false;
    bool in_joint_state = false;
    	
    init_action_type_t to_do = CARTESIAN;
    int i = 0;
    Trajectory<double> insert_tr;
    
    irob_msgs::RobotFeedback feedback;
    irob_msgs::RobotResult result;


	ROS_INFO_STREAM("Starting " << arm_typ.name << " initilaization");
  
    // Set robot state to cartasian
    bool set_state_cartasian_done = false;
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("InitRobotServerDVRK: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
		
		try {
			switch (to_do)
			{
      			case CARTESIAN:
      				feedback.info = "setting_cartesian";
      				success = setRobotState(STATE_POSITION_CARTESIAN);
      				break;
      				
      			case JOINT:
      				feedback.info = "setting_joint";
      				in_joint_state = setRobotState(STATE_POSITION_JOINT);
      				if (!in_joint_state) {
      					to_do = JOINT;
      				} else {
        				insert_tr = TrajectoryFactory::
        					linearTrajectoryWithSmoothAcceleration(
								getJointStateCurrent(INSERTION_JOINT_IDX),
								INSERTION_DEPTH, 
								INSERTION_SPEED,
								INSERTION_SPEED * 10.0, 
								INSERTION_DT);
						i = 0;
						to_do = INSERT;
      				}
      				break;
 
      			case INSERT:
      				
      				feedback.info = "inserting_tool";
      				moveJointAbsolute(INSERTION_JOINT_IDX, 
      								insert_tr[i], insert_tr.dt); 
      				i++;
      				if (i < insert_tr.size())
      					to_do = INSERT;
      				else
      					to_do = CARTESIAN;
      				
      				break;

      		}
        } catch (std::runtime_error e) {
        	std::size_t found 
        			= std::string(e.what()).find(ERROR_INSIDE_CANNULA);
        			
        	// If tool is inside cannula and movement is allowed, 
        	// then push tool in
        	if (found!=std::string::npos) 
        	{
        		if (move_allowed)
        		{
        			
        			to_do = JOINT;
        			ROS_INFO_STREAM(e.what());
        			ROS_INFO_STREAM("Attempting to move tool past the cannula");
        		}
        		else
        		{
        			ROS_ERROR_STREAM(e.what());
        			ROS_ERROR_STREAM("Cannot move tool past the cannula," 
        							<<" tool movement is not allowed");
        			result.pose = getPoseCurrent().toRosToolPose();
      				result.info = "cannot enable cartesian,movement not allowed";
      				// set the action state to succeeded
      				as.setAborted(result);
        			success = false;
        			break;
        		}
        	} 
        	// Unknown error occured, throw it
        	// TODO reconsider to return with failiure
        	else
        	{
        		//throw e;
        	}
		}
		
		as.publishFeedback(feedback);
		if (to_do != INSERT)
      		loop_rate.sleep();
      	else
      		insert_rate.sleep();
    }

    if(success)
    {
      	ROS_INFO_STREAM(arm_typ.name << " initilaization succeeded");
		result.info ="initilaization succeeded";
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
}


void RobotServerPSM::resetPose(bool move_allowed)
{
    // helper variables
    bool success = false;
    
    irob_msgs::RobotFeedback feedback;
    irob_msgs::RobotResult result;

	ROS_INFO_STREAM("Starting " << arm_typ.name << " pose reset");
  
    	// Check that preempt has not been requested by the client
    if (as.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO_STREAM(arm_typ.name << " pose reset: Preempted");
        // Set the action state to preempted
        as.setPreempted();
        success = false;
    }
  	
  	ROS_INFO_STREAM(arm_typ.name << " pose reset not implemented");
  	success = true;	
  	// Send some feedback
  	feedback.info = "done";
  	result.pose = getPoseCurrent().toRosToolPose();
    as.publishFeedback(feedback);

    if(success)
    {
      result.info = "done";
      result.pose = getPoseCurrent().toRosToolPose();
      ROS_INFO_STREAM(arm_typ.name << " pose reset succeded");
      // set the action state to succeeded
      as.setSucceeded(result);
    }
}

RobotServerPSM::~RobotServerPSM()
{
	// TODO Auto-generated destructor stub
}


void RobotServerPSM::advertiseLowLevelTopics() 
{
	RobotServerDVRK::advertiseLowLevelTopics();
	
	position_jaw_pub = nh.advertise<std_msgs::Float32>(
                    	TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/set_jaw_position"),
                        1000);
}


void RobotServerPSM::positionCartesianCurrentCB(
				const geometry_msgs::PoseStampedConstPtr& msg) 
{
  	position_cartesian_current = *msg;
	irob_msgs::ToolPoseStamped fwd;
	fwd.header = position_cartesian_current.header;


 	Pose tmp(position_cartesian_current, position_joint.position[6]);

	// Hand-eye calibration
	// Convert from m-s to mm-s
	fwd.pose = tmp.transform(R, t, 1000.0).toRosToolPose();
    position_cartesian_current_pub.publish(fwd);
}

Pose RobotServerPSM::getPoseCurrent()
{
 	ros::spinOnce();
 	Pose ret(position_cartesian_current, position_joint.position[6]);
 	return ret;
}

/*
 * DVRK actions
 */
void RobotServerPSM::moveJawRelative(double movement, double dt)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.jaw += movement;
    std_msgs::Float32 new_position_jaw = pose.toRosJaw();
    try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
     	} catch (std::runtime_error e)
  	{
  		std::string str_nan = "NaN";
  		std::string str_err = e.what();
  		if (str_err.find(str_nan) != std::string::npos)
    		ROS_ERROR_STREAM(e.what());
    	else
    		throw e;
  	}
}

void RobotServerPSM::moveJawAbsolute(double jaw, double dt)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.jaw = jaw;
    std_msgs::Float32 new_position_jaw = pose.toRosJaw();
    try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
     	} catch (std::runtime_error e)
  	{
  		std::string str_nan = "NaN";
  		std::string str_err = e.what();
  		if (str_err.find(str_nan) != std::string::npos)
    		ROS_ERROR_STREAM(e.what());
    	else
    		throw e;
  	}
}

void RobotServerPSM::moveCartesianAbsolute(Pose pose, double dt)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
    std_msgs::Float32 new_position_jaw = pose.toRosJaw();
    try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
     	} catch (std::runtime_error e)
  	{
  		std::string str_nan = "NaN";
  		std::string str_err = e.what();
  		if (str_err.find(str_nan) != std::string::npos)
    		ROS_ERROR_STREAM(e.what());
    	else
    		throw e;
  	}
}



}

using namespace ias;

/**
 * Main for RobotServerPSM
 */
int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "dvrk_il");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    std::string arm_typ;
	priv_nh.getParam("arm_typ", arm_typ);
	ArmTypes arm_type = ArmTypes::typeForString(arm_typ);

	std::string arm_name;
	priv_nh.getParam("arm_name", arm_name);
	
	std::string camera_registration_file;
	priv_nh.getParam("camera_registration_file", camera_registration_file);
	


	
    
    // Robot control
  	try {
  		if (arm_type == ArmTypes::PSM1 || arm_type == ArmTypes::PSM2) {
    		RobotServerPSM psm(nh, arm_type, 
    			 arm_name, camera_registration_file, RobotServerPSM::ACTIVE);
    		psm.initRosCommunication();
    		ros::spin();
    	}
    	else {
    		RobotServerDVRK arm(nh, arm_type, 
    			arm_name, camera_registration_file, RobotServerDVRK::ACTIVE);
    		arm.initRosCommunication();
    		ros::spin();
    	}  	   		    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}

