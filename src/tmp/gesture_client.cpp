/*
 *  gesture_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-18
 *  
 */

#include "irob_motion/gesture_server.hpp"

using namespace ias;


const std::string Arm::HOME_CMD
                    = "Home";
const std::string Arm::HOME_DONE
                    = "DVRK_READY";
const std::string Arm::STATE_POSITION_JOINT
                    = "DVRK_POSITION_JOINT";
                   // = "DVRK_POSITION_GOAL_JOINT";
const std::string Arm::STATE_POSITION_CARTESIAN
                    ="DVRK_POSITION_CARTESIAN";
                   // ="DVRK_POSITION_GOAL_CARTESIAN";
                   



GestureServer::GestureServer(ros::NodeHandle nh, std::string arm_name): 
			nh(nh), arm_name(arm_name),
			init_as(nh, "init_arm", boost::bind(
				&Arm::initArmActionCB, this, _1), false),
			reset_pose_as(nh, "reset_pose", boost::bind(
				&Arm::resetPoseActionCB, this, _1), false),
			follow_tr_as(nh,"follow_trajectory",boost::bind(
				&Arm::followTrajectoryActionCB, this, _1), false)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    startActionServers();
   
    
}

GestureServer::~GestureServer()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void GestureServer::initArmActionCB(const irob_autosurg::InitArmGoalConstPtr &goal)
{
    // helper variables
    ros::Rate loop_rate(2);
    bool success = false;

    irob_autosurg::InitArmFeedback feedback;
    irob_autosurg::InitArmResult result;


	ROS_INFO_STREAM("Starting " << arm_typ.name << " initilaization");
  
    Eigen::Matrix3d p = Eigen::Matrix3d::Random(3,3);
    p = p.transpose();

    // Set robot state to cartasian
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (init_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm_typ.name << " initilaization: Preempted");
        	// Set the action state to preempted
        	init_as.setPreempted();
        	success = false;
        	break;
      	}
		
		try {
      		success	= setRobotState(STATE_POSITION_CARTESIAN);
        } catch (std::runtime_error e) { 
        	// Unknown error occured, stop action and throw it
        	throw e;
        } 
  		
  		// Send some feedback
  		feedback.status = "setting_cartesian";
		feedback.info = arm_typ.name + " attempting to start cartesian mode";
      	init_as.publishFeedback(feedback);
      	// this sleep is not necessary
      	loop_rate.sleep();
    }

    if(success)
    {
      	result.descript = robot_state.data;
      	ROS_INFO_STREAM(arm_typ.name << " initilaization succeeded");
		result.info = arm_typ.name + " initilaization succeeded";
      	// set the action state to succeeded
      	init_as.setSucceeded(result);
    }
}
  
void GestureServer::resetPoseActionCB(const irob_autosurg::ResetPoseGoalConstPtr &goal)
{
    // helper variables
    bool success = false;
    
    irob_autosurg::ResetPoseFeedback feedback;
    irob_autosurg::ResetPoseResult result;

	ROS_INFO_STREAM("Starting " << arm_typ.name << " pose reset");
  
    	// Check that preempt has not been requested by the client
    if (reset_pose_as.isPreemptRequested() || !ros::ok())
    {
        ROS_INFO_STREAM(arm_typ.name << " pose reset: Preempted");
        // Set the action state to preempted
        reset_pose_as.setPreempted();
        success = false;
    }
  	
  	ROS_INFO_STREAM(arm_typ.name << " pose reset not implemented");
  	success = true;	
  	// Send some feedback
  	feedback.status = "done";
    reset_pose_as.publishFeedback(feedback);

    if(success)
    {
      result.descript = "done";
      ROS_INFO_STREAM(arm_typ.name << " pose reset succeded");
      // set the action state to succeeded
      reset_pose_as.setSucceeded(result);
    }
}
  
  
void GestureServer::followTrajectoryActionCB(
		const irob_autosurg::FollowTrajectoryGoalConstPtr &goal)
  {
    // helper variables
    bool success = true;
    irob_autosurg::FollowTrajectoryFeedback feedback;
    irob_autosurg::FollowTrajectoryResult result;

	ROS_INFO_STREAM("Starting trajectory follow action.");
	
	Trajectory<Pose> tr(goal->trajectory);
	// TODO hande-eye calibration
	
	ros::Rate loop_rate(1.0/tr.dt);
	// start executing the action
	for (int i = 0; i < tr.size(); i++)
	{
		// check that preempt has not been requested by the client
      	if (follow_tr_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Follow trajectory: Preempted");
       	 	// set the action state to preempted
        	follow_tr_as.setPreempted();
        	success = false;
        	break;
      	}
		moveCartesianAbsolute(tr[i],tr.dt);
		
		feedback.pose = tr[i].toRosToolPose();
      	follow_tr_as.publishFeedback(feedback);
		
		loop_rate.sleep();
	}
	
    if(success)
    {
      result.pose = getPoseCurrent().toRosToolPose();
      ROS_INFO_STREAM("Follow trajectory: Succeeded");
      // set the action state to succeeded
      follow_tr_as.setSucceeded(result);
    }
  }


void GestureServer::positionCartesianCurrentCB(
				const geometry_msgs::PoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
	irob_autosurg::ToolPoseStamped fwd;
	fwd.header = position_cartesian_current.header;

 	Pose tmp(position_cartesian_current, 0.0);

	// TODO hand-eye calibration
	
	fwd.pose = tmp.toRosToolPose();
    position_cartesian_current_pub.publish(fwd);
}


void GestureServer::subscribeTopics() 
{
	robot_state_sub = nh.subscribe<std_msgs::String>(
                        TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/robot_state"),
                       	1000, &Arm::robotStateCB,this);
  
  	state_joint_current_sub = nh.subscribe<sensor_msgs::JointState>(
                        TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/state_joint_current"),
                       	1000, &Arm::stateJointCurrentCB,this);  
                       	            	
   	position_cartesian_current_sub = nh.subscribe<geometry_msgs::PoseStamped>(
                        TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/position_cartesian_current"),
                       	1000, &Arm::positionCartesianCurrentCB,this);
                       	
	error_sub = nh.subscribe<std_msgs::String>(
                        TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/error"),
                       	1000, &Arm::errorCB,this);
                       	
    warning_sub = nh.subscribe<std_msgs::String>(
                        TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/warning"),
                       	1000, &Arm::warningCB,this);

}

void GestureServer::advertiseTopics() 
{
	// dVRK
	robot_state_pub = nh.advertise<std_msgs::String>(
                    	TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/set_robot_state"),
                        1000);
    position_joint_pub = nh.advertise<sensor_msgs::JointState>(
                    	TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/set_position_joint"),
                        1000);
   	position_cartesian_pub = nh.advertise<geometry_msgs::Pose>(
                    	TopicNameLoader::load(nh,
                        	"dvrk_topics/namespace",
                        	arm_typ.name,
                        	"dvrk_topics/set_position_cartesian"),
                        1000);  

	// robot interface
	position_cartesian_current_pub 
				= nh.advertise<irob_autosurg::ToolPoseStamped>(
                    	"position_cartesian_current",
                        1000);   
}

void GestureServer::startActionServers() 
{

	init_as.start();
	reset_pose_as.start();
    follow_tr_as.start();
}



Pose GestureServer::getPoseCurrent()
{
 	ros::spinOnce();
 	Pose ret(position_cartesian_current, 0.0);
 	return ret;

}



