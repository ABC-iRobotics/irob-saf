/*
 *  psm.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-17
 *  
 */

#include "irob_dvrk/psm.hpp"
#include <numeric>
#include <chrono>

using namespace irob_autosurg;

namespace irob_dvrk {

PSM::PSM(ros::NodeHandle nh, ArmTypes arm_typ, bool isActive): Arm(nh, arm_typ, isActive)
{
	if (!(arm_typ == ArmTypes::PSM1 || 
  							arm_typ == ArmTypes::PSM2))
  		throw std::runtime_error(
  		"Tried to create PSM object for ECM or MTM arm type.");
  			
   	advertise(Topics::SET_POSITION_JAW);
    
}

PSM::~PSM()
{
	// TODO Auto-generated destructor stub
}


bool PSM::advertise(Topics topic) 
{
	if(topic == Topics::SET_POSITION_JAW)
    {
        position_jaw_pub = nh.advertise<std_msgs::Float32>(
                                   topic.getFullName(arm_typ), 1000);
        ROS_DEBUG_STREAM("Advertised topic " << topic.getFullName(arm_typ));
        return true;
    }
    return Arm::advertise(topic);
}

Pose PSM::getPoseCurrent()
{
 	ros::spinOnce();
 	Pose ret(position_cartesian_current, position_joint.position[6]);
 	return ret;
}

/*
 * DVRK actions
 */
void PSM::moveJawRelative(double movement, double dt)
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

void PSM::moveJawAbsolute(double jaw, double dt)
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

void PSM::moveCartesianAbsolute(Pose pose, double dt)
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


