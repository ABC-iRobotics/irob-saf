/*
 * 	arm.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *
 */

#ifndef DVRK_ARM_HPP_
#define DVRK_ARM_HPP_

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
#include "irob_dvrk/arm_types.hpp"
#include "irob_utils/pose.hpp"
#include "irob_utils/trajectory.hpp"
#include "irob_utils/utils.hpp"
#include "irob_utils/topic_name_loader.hpp"
#include <actionlib/server/simple_action_server.h>
#include <irob_autosurg/ToolPoseStamped.h>
#include <irob_autosurg/InitArmAction.h>
#include <irob_autosurg/ResetPoseAction.h>
#include <irob_autosurg/FollowTrajectoryAction.h>

namespace ias {

class Arm {

public:
    // Constants
    static const std::string HOME_CMD;
    static const std::string HOME_DONE;
    static const std::string STATE_POSITION_JOINT;
    static const std::string STATE_POSITION_CARTESIAN;
    
    static const bool ACTIVE = true;
    static const bool PASSIVE = false;
   

protected:
    const ArmTypes arm_typ;
	const std::string arm_name;
    ros::NodeHandle nh;
    
    // Action servers
    actionlib::SimpleActionServer<irob_autosurg::InitArmAction>
    	 init_as;
   	actionlib::SimpleActionServer<irob_autosurg::ResetPoseAction>
    	 reset_pose_as;
    actionlib::SimpleActionServer<irob_autosurg::FollowTrajectoryAction>
    	 follow_tr_as;
   	
    // Hand-eye registration
    Eigen::Vector3d t;
    Eigen::Matrix3d R;

    // States
    std_msgs::String robot_state;
    sensor_msgs::JointState position_joint;
    geometry_msgs::PoseStamped position_cartesian_current;
    std_msgs::String error;
    std_msgs::String warning;

    // Subscribers
    ros::Subscriber robot_state_sub;
    ros::Subscriber state_joint_current_sub;
    ros::Subscriber position_cartesian_current_sub;
    ros::Subscriber error_sub;
    ros::Subscriber warning_sub;


    // Publishers
    ros::Publisher robot_state_pub;
    ros::Publisher position_joint_pub;
    ros::Publisher position_cartesian_pub;

	ros::Publisher position_cartesian_current_pub;
    
    void subscribeTopics();
    void advertiseTopics();
    void startActionServers();

public:
	Arm(ros::NodeHandle, ArmTypes, std::string, std::string, bool);
	~Arm();

    // Callbacks
    virtual void initArmActionCB(const irob_autosurg::InitArmGoalConstPtr &);
    virtual void resetPoseActionCB(const irob_autosurg::ResetPoseGoalConstPtr &);
    void followTrajectoryActionCB(
    		const irob_autosurg::FollowTrajectoryGoalConstPtr &);
    
    void robotStateCB(const std_msgs::String);
    void errorCB(const std_msgs::String);
    void warningCB(const std_msgs::String);
    void stateJointCurrentCB(const sensor_msgs::JointStateConstPtr&);
    virtual void positionCartesianCurrentCB(const geometry_msgs::PoseStampedConstPtr&);

   
    void loadRegistration(std::string);
    
    double getJointStateCurrent(int);
    std::vector<double> getJointStateCurrent();
    Eigen::Vector3d getPositionCartesianCurrent();
    Eigen::Quaternion<double> getOrientationCartesianCurrent();
    virtual Pose getPoseCurrent();

    //DVRK actions
    bool setRobotState(std::string);
    void moveJointRelative(int, double, double = 0.01 );
    void moveJointAbsolute(int, double, double = 0.01 );
    void moveCartesianRelative(Eigen::Vector3d, double = 0.01);
    void moveCartesianAbsolute(Eigen::Vector3d, double = 0.01);
    void moveCartesianAbsolute(Eigen::Quaternion<double>, double = 0.01);
    virtual void moveCartesianAbsolute(Pose, double = 0.01);
 
	
	void recordTrajectory(Trajectory<Eigen::Vector3d>&);
	void recordTrajectory(Trajectory<Pose>&);
	
	void checkErrors();
	void checkVelCartesian(const Pose&, const Pose&, double);
	void checkNaNCartesian(const Pose&);
	void checkVelJoint(const sensor_msgs::JointState&, 
						const std::vector<double>&, double);
	void checkNaNJoint(const sensor_msgs::JointState&);
	
};

}
#endif /* DVRK_ARM_HPP_ */
