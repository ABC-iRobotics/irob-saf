/*
 * 	robot_server.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
 * 
 *  Important note: initRosCommunication must be called 
 *  in child class or main function!
 *
 */

#ifndef ROBOT_SERVER_HPP_
#define ROBOT_SERVER_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include <irob_utils/pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/topic_name_loader.hpp>
#include <actionlib/server/simple_action_server.h>
#include <irob_msgs/ToolPoseStamped.h>
#include <irob_msgs/RobotAction.h>

namespace ias {

class RobotServer {
   
public:
	// Constants
	static const bool ACTIVE = true;
    static const bool PASSIVE = false;

protected:

	const std::string arm_name;
    ros::NodeHandle nh;
    bool isActive;
    
    // Action servers
    actionlib::SimpleActionServer<irob_msgs::RobotAction> as;
    	 
    // Hand-eye registration
    Eigen::Vector3d t;
    Eigen::Matrix3d R;

	// Publisher
	ros::Publisher position_cartesian_current_pub;
    
    virtual void subscribeLowLevelTopics() = 0;
    virtual void advertiseLowLevelTopics() = 0;
    
    void advertiseHighLevelTopics()
    {
    	// robot interface
		position_cartesian_current_pub 
				= nh.advertise<irob_msgs::ToolPoseStamped>(
                    	"robot/"+arm_name+"/position_cartesian_current_cf",
                        1000);
    }
    
    void startActionServer()
    {
		as.start();
	}
	
	

public:

	void initRosCommunication()
	{
		subscribeLowLevelTopics();
    	if (isActive == ACTIVE)
    		advertiseLowLevelTopics();
    		
		advertiseHighLevelTopics();
		startActionServer();
	}
	
	void loadRegistration(std::string registration_file)
    {
    	std::ifstream cfgfile(registration_file.c_str());
    	if (!cfgfile.is_open())
    		throw std::runtime_error(
    			"Cannot open file " + registration_file);
    	if (cfgfile.eof())
    		throw std::runtime_error(
    			"Cfgfile " + registration_file + " is empty.");
   	
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
    
    	ROS_INFO_STREAM(
    		"Registration read: "<< std::endl << t << std::endl << R);
    }
    
    
    
    virtual void initArm(bool) = 0;
    virtual void resetPose(bool) = 0;
    virtual void stop() = 0;
    virtual void followTrajectory(Trajectory<Pose>) = 0;
    
    virtual Pose getPoseCurrent() = 0;

    
    virtual void robotActionCB(const irob_msgs::RobotGoalConstPtr& goal)
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
	
	// initRosCommunication must be called in child class or main function
	RobotServer(ros::NodeHandle nh, std::string arm_name, std::string regfile,
				bool isActive): 
		nh(nh), arm_name(arm_name), isActive(isActive),
		as(nh, "robot/"+arm_name+"/robot_action",
			boost::bind(&RobotServer::robotActionCB, this, _1))
	{
		loadRegistration(regfile);
	}
	
	~RobotServer() {}

    
};

}
#endif /* ROBOT_SERVER_HPP_ */
