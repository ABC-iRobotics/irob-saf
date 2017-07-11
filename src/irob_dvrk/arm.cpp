/*
 *  dvrk_arm.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *  
 */

#include "irob_dvrk/arm.hpp"
#include <numeric>
#include <chrono>

using namespace irob_autosurg;

namespace irob_dvrk {

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
                   
const std::string Arm::ERROR_NOT_READY
                    ="is not ready";
                   
const std::string Arm::ERROR_INSIDE_CANNULA
                    ="make sure the tool is inserted past the cannula";


Arm::Arm(ros::NodeHandle nh, ArmTypes arm_typ, bool isActive): 
			nh(nh), arm_typ(arm_typ),
			init_as(nh, "init_arm", boost::bind(
				&Arm::initArmActionCB, this, _1), false),
			reset_pose_as(nh, "reset_pose", boost::bind(
				&Arm::resetPoseActionCB, this, _1), false),
			follow_tr_as(nh,"follow_trajectory",boost::bind(
				&Arm::followTrajectoryActionCB, this, _1), false)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    if (isActive == ACTIVE)
    	advertiseTopics();
    startActionServers();
   
    
}

Arm::~Arm()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void Arm::initArmActionCB(const irob_autosurg::InitArmGoalConstPtr &goal)
{
    // helper variables
    ros::Rate loop_rate(2);
    bool success = true;
    typedef enum init_action_type 
    	{CARTESIAN, HOME, INSERT, STOP} 
    	init_action_type_t;
    	
    init_action_type_t to_do = CARTESIAN;
    
    irob_autosurg::InitArmFeedback feedback;
    irob_autosurg::InitArmResult result;


	ROS_INFO_STREAM("Starting InitArm action.");
  
    // Set robot state to cartasian
    bool set_state_cartasian_done = false;
    while(!set_state_cartasian_done)
    {
    	// Check that preempt has not been requested by the client
      	if (init_as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("InitArm: Preempted");
        	// Set the action state to preempted
        	init_as.setPreempted();
        	success = false;
        	break;
      	}
		
		try {
			switch (to_do)
			{
      			case CARTESIAN:
      				set_state_cartasian_done 
      						= setRobotState(STATE_POSITION_CARTESIAN);
      				break;
      			case HOME:
      				if (home())
      					to_do = CARTESIAN;
      				break;
      			case INSERT: 
      				break;
      			case STOP: 
      				break;
      		}
        } catch (std::runtime_error e) {
        	
        	// If arm is not homed, then home
        	std::size_t not_ready_found 
        			= std::string(e.what()).find(ERROR_NOT_READY);
        	std::size_t inside_cannula_found 
        			= std::string(e.what()).find(ERROR_INSIDE_CANNULA);
  			if (not_ready_found!=std::string::npos)
  			{
  				to_do = HOME;
  				ROS_INFO_STREAM(e.what() << ", attempting to home arm");
  			}
        	// If tool is inside cannula and movement is allowed, 
        	// then push tool in
        	else if (inside_cannula_found!=std::string::npos) 
        	{
        		if (goal->move_allowed)
        		{
        			to_do = INSERT;
        			ROS_INFO_STREAM(e.what() 
        				<< ", attempting to move tool past the cannula");
        		}
        		else
        		{
        			to_do = STOP;
        			ROS_ERROR_STREAM(e.what() 
        				<< ", tool movement is not allowed");
        		}
        	} 
        	// Unknown error occured, throw it
        	else
        	{
        		throw e;
        	}
        } 


      
     feedback.status = "Doing Home ...";
      init_as.publishFeedback(feedback);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      loop_rate.sleep();
    }

    if(success)
    {
      result.descript = "Home done";
      ROS_INFO_STREAM("Home: Succeeded");
      // set the action state to succeeded
      init_as.setSucceeded(result);
    }
  }
  
void Arm::resetPoseActionCB(const irob_autosurg::ResetPoseGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(1);
    bool success = true;
    
    irob_autosurg::ResetPoseFeedback feedback;
    irob_autosurg::ResetPoseResult result;


	ROS_INFO_STREAM("Starting Home action.");
  
    // start executing the action
    for(int i=1; i<=10; i++)
    {
      // check that preempt has not been requested by the client
      if (reset_pose_as.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO_STREAM("Home: Preempted");
        // set the action state to preempted
        reset_pose_as.setPreempted();
        success = false;
        break;
      }
     feedback.status = "Doing Home ...";
      reset_pose_as.publishFeedback(feedback);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result.descript = "Home done";
      ROS_INFO_STREAM("Home: Succeeded");
      // set the action state to succeeded
      reset_pose_as.setSucceeded(result);
    }
}
  
  
void Arm::followTrajectoryActionCB(
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


 
void Arm::robotStateCB(const std_msgs::String msg)
{
    robot_state = msg;
}

void Arm::stateJointCurrentCB(const sensor_msgs::JointStateConstPtr& msg) 
{
    position_joint = *msg;
}

void Arm::positionCartesianCurrentCB(
				const geometry_msgs::PoseStampedConstPtr& msg) 
{
     position_cartesian_current = *msg;
}

void Arm::errorCB(const std_msgs::String msg) 
{
     error = msg;
}

void Arm::warningCB(const std_msgs::String msg) 
{
     warning = msg;
}

void Arm::subscribeTopics() 
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

void Arm::advertiseTopics() 
{
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
}

void Arm::startActionServers() 
{

	init_as.start();
	reset_pose_as.start();
    follow_tr_as.start();
}

/*
 * DVRK actions
 */
double Arm::getJointStateCurrent(int index)
{
	ros::spinOnce();
	if (index > ((int)position_joint.position.size())-1)
	{
		ROS_WARN_STREAM("Joint index too big or no joint position information.");
		return NAN;
	}
 	return position_joint.position[index];
}

std::vector<double> Arm::getJointStateCurrent()
{
	ros::spinOnce();
	std::vector<double> ret(arm_typ.dof);
	
		ret = position_joint.position;
 	return ret;
}
 
Eigen::Vector3d Arm::getPositionCartesianCurrent()
{
 	ros::spinOnce();
 	Eigen::Vector3d ret(position_cartesian_current.pose.position.x,
 						position_cartesian_current.pose.position.y,
 						position_cartesian_current.pose.position.z);
    return ret;
}

Eigen::Quaternion<double> Arm::getOrientationCartesianCurrent()
{
	ros::spinOnce();
 	Eigen::Quaternion<double> ret(position_cartesian_current.pose.orientation.x,
 		position_cartesian_current.pose.orientation.y, 
 		position_cartesian_current.pose.orientation.z, 
 		position_cartesian_current.pose.orientation.w);
    return ret;
}

Pose Arm::getPoseCurrent()
{
 	ros::spinOnce();
 	Pose ret(position_cartesian_current, 0.0);
 	return ret;

}


bool Arm::home()
{
	std_msgs::String msg;
	std::stringstream ss;
    ss << HOME_CMD;
    msg.data = ss.str();
    robot_state_pub.publish(msg);
    ros::Duration(0.5).sleep();
	ros::spinOnce();
    if (robot_state.data == HOME_DONE) {
    	ROS_INFO_STREAM("State set to Home");
        return true;
   	}
   	
    ros::spinOnce();
    return false;
}

bool Arm::setRobotState(std::string state)
{
	std_msgs::String msg;
    std::stringstream ss;
    ss << state;
    msg.data = ss.str();
    robot_state_pub.publish(msg);
    ros::Duration(0.5).sleep();
	ros::spinOnce();
	checkErrors();
    if (robot_state.data == state) {
    	ROS_INFO_STREAM("State set to " << state);
        return true;
    }
    
  	ROS_INFO_STREAM("State set to " << robot_state.data);
    ros::spinOnce();
    return false;
}

void Arm::moveJointRelative(int joint_idx, double movement, double dt)
{
   	// Collect data
    std::vector<double> currJoint = getJointStateCurrent();
    sensor_msgs::JointState new_position_joint = position_joint;
    new_position_joint.position[joint_idx] += movement;
    
    // Safety
    try {
    checkErrors();
    checkVelJoint(new_position_joint, currJoint, dt);
    checkNaNJoint(new_position_joint);
    // End safety
    
    // Publish movement
    position_joint_pub.publish(new_position_joint);
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

void Arm::moveJointAbsolute(int joint_idx, double pos, double dt)
{
	// Collect data
    std::vector<double> currJoint = getJointStateCurrent();
    sensor_msgs::JointState new_position_joint = position_joint;
    new_position_joint.position[joint_idx] = pos;
    
   	// Safety
   	try {
    checkErrors();
    checkVelJoint(new_position_joint, currJoint, dt);
    checkNaNJoint(new_position_joint);
    // End safety
    
    // Publish movement
    position_joint_pub.publish(new_position_joint);
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

void Arm::moveCartesianRelative(Eigen::Vector3d movement, double dt)
{
    // Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose += movement;
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
  	
  	// Safety
  	try {
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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
    //ros::Duration(0.1).sleep();

}

void Arm::moveCartesianAbsolute(Eigen::Vector3d position, double dt)
{
    // Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.position = position;
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
    
    // Safety
    try {
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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
    //ros::Duration(0.1).sleep();

}

void Arm::moveCartesianAbsolute(Eigen::Quaternion<double> orientation, double dt)
{
	// Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.orientation = orientation;
   	geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
   	
    // Safety
    try {
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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

void Arm::moveCartesianAbsolute(Pose pose, double dt)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
    try {
    // Safety
    checkErrors();
    checkVelCartesian(pose, currPose, dt);
    checkNaNCartesian(pose);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
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

void Arm::checkErrors()
{
	if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
}

void Arm::checkVelCartesian(const Pose& pose, 
								const Pose& currPose, double dt)
{
	Pose::Distance d = currPose.dist(pose)/dt;
    if (d.cartesian > arm_typ.maxVelPose.cartesian ||	
    	d.angle > arm_typ.maxVelPose.angle ||  
    	d.jaw > arm_typ.maxVelPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Velocity:\t" << d << std::endl
    			<< "MaxVelocity:\t" << arm_typ.maxVelPose << std::endl;
    	//ROS_ERROR_STREAM(errstring.str());
		throw std::runtime_error(errstring.str());
	}
}

void Arm::checkNaNCartesian(const Pose& pose)
{
    if (pose.isNaN())
    {
    	std::stringstream errstring;
    	errstring << "Desired pose is NaN:\t"<< std::endl
    			<< pose << std::endl;
    	//ROS_ERROR_STREAM(errstring.str());
		throw std::runtime_error(errstring.str());
	}
}

void Arm::checkVelJoint(const sensor_msgs::JointState& new_position_joint, 
						const std::vector<double>& currJoint, double dt)
{
	std::vector<double> distance(arm_typ.dof);
    for (int i = 0; i < arm_typ.dof; i++) {
    	distance[i] = std::abs(new_position_joint.position[i]-currJoint[i])/dt;
    }
    for (int i = 0; i < arm_typ.dof; i++)
    { 		
    	if (distance[i] > arm_typ.maxVelJoint[i])
   		{
    		std::stringstream errstring;
    		errstring << "Desired joint vector too far from current." 
    					<< std::endl
    					<< "Desired joint vector:\t" 
    					<< new_position_joint.position
    					<< std::endl
    					<< "Current joint vector:\t" 
    					<< currJoint
    					<< std::endl
    					<< "Velocity:\t" 
    					<< distance
    					<< std::endl
    					<< "MaxVelocity:\t" 
    					<< arm_typ.maxVelJoint
    					<< std::endl;
    		//ROS_ERROR_STREAM(errstring.str());
			throw std::runtime_error(errstring.str());
		}
	}
}

void Arm::checkNaNJoint(const sensor_msgs::JointState& new_position_joint)
{
	bool foundNaN = false;
    for (int i = 0; i < arm_typ.dof; i++) {
    	foundNaN = foundNaN || std::isnan(new_position_joint.position[i]);
    }		
    if (foundNaN)
   	{
    	std::stringstream errstring;
    	errstring << "Desired joint vector is NaN:\t" 
    					<< new_position_joint.position
    					<< std::endl;
    		//ROS_ERROR_STREAM(errstring.str());
			throw std::runtime_error(errstring.str());
		}
}

/*
 * Trajectories
 */
void Arm::playTrajectory(Trajectory<Eigen::Vector3d>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	//int cnt = 0;
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		//auto start = std::chrono::high_resolution_clock::now();
		
		moveCartesianAbsolute(tr[i],tr.dt);
		
		loop_rate.sleep();
		/*
		std::chrono::duration<double> elapsed =
	 			std::chrono::high_resolution_clock::now()-start;
	 	cnt ++;
	 	if (cnt >= 10)
	 	{
			ROS_INFO("Time elapsed: %f us", (elapsed*1000000.0));
			cnt = 0;
		}*/
	}

}

void Arm::playTrajectory(Trajectory<Eigen::Quaternion<double>>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(tr[i],tr.dt);
		loop_rate.sleep();
	}
}

void Arm::playTrajectory(Trajectory<Pose>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(tr[i],tr.dt);
		loop_rate.sleep();
	}
}

void Arm::playTrajectory(int jointIndex, Trajectory<double>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveJointAbsolute(jointIndex, tr[i],tr.dt);
		loop_rate.sleep();
	}
}

void Arm::recordTrajectory(Trajectory<Eigen::Vector3d>& tr) 
{
	ros::Rate loop_rate(1.0/tr.dt);
	// Skip invalid points
	while (ros::ok() && getPositionCartesianCurrent().norm() < 0.001)
  	{
  		loop_rate.sleep();
  	}
	while(ros::ok())
	{
		tr.addPoint(getPositionCartesianCurrent());
		loop_rate.sleep();
	}
}

void Arm::recordTrajectory(Trajectory<Pose>& tr) 
{
	ros::Rate loop_rate(1.0/tr.dt);
	// Skip invalid points
	while (ros::ok() && getPositionCartesianCurrent().norm() < 0.001)
  	{
  		loop_rate.sleep();
  	}
	while(ros::ok())
	{
		tr.addPoint(getPoseCurrent());
		loop_rate.sleep();
	}
}


}



