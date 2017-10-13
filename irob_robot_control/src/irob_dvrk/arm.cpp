/*
 *  dvrk_arm.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *  
 */

#include <irob_dvrk/arm.hpp>
#include <numeric>
#include <chrono>

namespace ias {


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
                   



Arm::Arm(ros::NodeHandle nh, ArmTypes arm_typ, std::string arm_name, 
									std::string regfile, bool isActive): 
			nh(nh), arm_typ(arm_typ), arm_name(arm_name),
			as(nh, "robot/"+arm_name+"/robot_action", boost::bind(
				&Arm::robotActionCB, this, _1), false)
{
	loadRegistration(regfile);
	// Subscribe and advertise topics
	subscribeTopics();
    if (isActive == ACTIVE)
    	advertiseTopics();
    startActionServer();
}

Arm::~Arm()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void Arm::robotActionCB(const irob_msgs::RobotGoalConstPtr &goal)
{
    switch(goal -> action)
    {
    	case irob_msgs::RobotGoal::STOP:
    	{
    		stop();
    		break;
    	}	
    		
    	case  irob_msgs::RobotGoal::INIT_ARM:
    	{
    		initArm(goal -> move_allowed);
    		break;
    	}
    	
    	case  irob_msgs::RobotGoal::RESET_POSE:
    	{
    		resetPose(goal -> move_allowed);
    		break;
    	}	
    		
    	case  irob_msgs::RobotGoal::FOLLOW_TRAJECTORY:
    	{
    		followTrajectory(goal->trajectory);
    		break;
    	}
    	default:
    	{
    		ROS_ERROR_STREAM(arm_name  << 
    					": invalid robot action code received");
    		irob_msgs::RobotResult result;
    		result.pose = getPoseCurrent().toRosToolPose();
			result.info = "invalid robot action code";
      		as.setAborted(result);
    		break;
    	}    	
    }	
}
 
void Arm::initArm(bool move_allowed)
{
    // helper variables
    ros::Rate loop_rate(2);
    bool success = false;

    irob_msgs::RobotFeedback feedback;
    irob_msgs::RobotResult result;


	ROS_INFO_STREAM("Starting " << arm_typ.name << " initilaization");

    // Set robot state to cartasian
    while(!success)
    {
    	// Check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM(arm_typ.name << " initilaization: Preempted");
        	// Set the action state to preempted
        	as.setPreempted();
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
		feedback.info = "setting cartesian";
		feedback.pose = getPoseCurrent().toRosToolPose();
      	as.publishFeedback(feedback);
      	// this sleep is not necessary
      	loop_rate.sleep();
    }

    if(success)
    {
		result.info = "initilaization succeeded";
		result.pose = getPoseCurrent().toRosToolPose();
      	// set the action state to succeeded
      	as.setSucceeded(result);
    }
}
  
void Arm::resetPose(bool move_allowed)
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
  	feedback.pose = getPoseCurrent().toRosToolPose();
    as.publishFeedback(feedback);

    if(success)
    {
      result.info = "done";
      result.pose = getPoseCurrent().toRosToolPose();
      ROS_INFO_STREAM(arm_typ.name << " pose reset succeeded");
      // set the action state to succeeded
      as.setSucceeded(result);
    }
}

void Arm::stop()
{
     // helper variables
    bool success = false;
    
    irob_msgs::RobotFeedback feedback;
    irob_msgs::RobotResult result;

	ROS_INFO_STREAM("Stopping " << arm_typ.name);
  
   	// Do not check that preempt has not been requested by the client
  	
  	success = true;	
  	// Send some feedback
  	feedback.info = "stopped";
  	feedback.pose = getPoseCurrent().toRosToolPose();
    as.publishFeedback(feedback);

    if(success)
    {
      result.info = "stopped";
      result.pose = getPoseCurrent().toRosToolPose();
      ROS_INFO_STREAM(arm_typ.name << " stopped");
      // set the action state to succeeded
      as.setSucceeded(result);
    }
}
  
  
  
void Arm::followTrajectory(Trajectory<Pose> tr)
  {
    // helper variables
    bool success = true;
    irob_msgs::RobotFeedback feedback;
    irob_msgs::RobotResult result;

	ROS_INFO_STREAM("Starting trajectory follow action.");
	
	// Go to m-s from mm-s here
	// Hande-eye calibration
	tr.invTransform(R, t, 1000.0);
	
	
	ros::Rate loop_rate(1.0/tr.dt);
	// start executing the action
	for (int i = 0; i < tr.size(); i++)
	{
		// check that preempt has not been requested by the client
      	if (as.isPreemptRequested() || !ros::ok())
      	{
        	ROS_INFO_STREAM("Follow trajectory: Preempted");
       	 	// set the action state to preempted
        	as.setPreempted();
        	success = false;
        	break;
      	}
      	try {
			moveCartesianAbsolute(tr[i],tr.dt);
			feedback.info = "following trajectory, index: " + i;
			feedback.pose = tr[i].toRosToolPose();
      		as.publishFeedback(feedback);
      	}catch (std::runtime_error e)
  		{
  			result.pose = getPoseCurrent().toRosToolPose();
      		result.info = e.what();
      		ROS_ERROR_STREAM(arm_typ.name << ": an error occured: " << e.what());
      		// set the action state to succeeded
      		as.setAborted(result);
  			success = false;
        	break;
  		}
  		
		loop_rate.sleep();
	}
	
    if(success)
    {
      result.pose = getPoseCurrent().toRosToolPose();
      result.info = "done";
      ROS_INFO_STREAM("Follow trajectory: Succeeded");
      // set the action state to succeeded
      as.setSucceeded(result);
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
	irob_msgs::ToolPoseStamped fwd;
	fwd.header = position_cartesian_current.header;

 	Pose tmp(position_cartesian_current, 0.0);

	// Hand-eye calibration
	// Convert from m-s to mm-s
	fwd.pose = tmp.transform(R, t, 1000.0).toRosToolPose();
    position_cartesian_current_pub.publish(fwd);
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
				= nh.advertise<irob_msgs::ToolPoseStamped>(
                    	"robot/"+arm_name+"/position_cartesian_current_cf",
                        1000); 
}

void Arm::startActionServer() 
{
	as.start();
}


void Arm::loadRegistration(std::string registration_file)
{
	std::ifstream cfgfile(registration_file.c_str());
    if (!cfgfile.is_open())
    	throw std::runtime_error("Cannot open file " + registration_file);
    if (cfgfile.eof())
    	throw std::runtime_error("Cfgfile " + registration_file + " is empty.");
   	
   	double x, y, z;
   	
    cfgfile >> x >> std::ws >> y >> std::ws >> z >> std::ws;
    t << x, y, z;
    
    for (int i = 0; i < 3; i++)
    {
    	cfgfile >> x >> std::ws >> y >> std::ws >> z >> std::ws;
    	R(i,0) = x;
    	R(i,1) = y;
    	R(i,2) = z;
    }
    
    cfgfile.close();
    
    ROS_INFO_STREAM("Registration read: "<< std::endl << t << std::endl << R);
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
    {
    	std::string err_data = error.data;
    	error.data.clear();
   		throw std::runtime_error(err_data);
   	}
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

