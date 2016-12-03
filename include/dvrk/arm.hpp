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
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include "dvrk/arm_types.hpp"
#include "dvrk/topics.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory.hpp"
#include "dvrk/utils.hpp"


namespace dvrk {

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
    ros::NodeHandle nh;

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
    
    bool subscribe(const Topics);
    bool advertise(const Topics);

public:
	Arm(ros::NodeHandle, ArmTypes, bool);
	~Arm();

    // Callbacks
    void robotStateCB(const std_msgs::String);
    void errorCB(const std_msgs::String);
    void warningCB(const std_msgs::String);
    void stateJointCurrentCB(const sensor_msgs::JointStateConstPtr&);
    void positionCartesianCurrentCB(const geometry_msgs::PoseStampedConstPtr&);

   
    
    double getJointStateCurrent(int);
    std::vector<double> getJointStateCurrent();
    Eigen::Vector3d getPositionCartesianCurrent();
    Eigen::Quaternion<double> getOrientationCartesianCurrent();
    virtual Pose getPoseCurrent();

    //DVRK actions
    bool home();
    bool setRobotState(std::string);
    void moveJointRelative(int, double, double = 0.01 );
    void moveJointAbsolute(int, double, double = 0.01 );
    void moveCartesianRelative(Eigen::Vector3d, double = 0.01);
    void moveCartesianAbsolute(Eigen::Vector3d, double = 0.01);
    void moveCartesianAbsolute(Eigen::Quaternion<double>, double = 0.01);
    virtual void moveCartesianAbsolute(Pose, double = 0.01);
    

	void playTrajectory(Trajectory<Eigen::Vector3d>&);
	void playTrajectory(Trajectory<Eigen::Quaternion<double>>&);
	void playTrajectory(Trajectory<Pose>&);
	void playTrajectory(int, Trajectory<double>&);
	
	void recordTrajectory(Trajectory<Eigen::Vector3d>&);
	void recordTrajectory(Trajectory<Pose>&);
	
	void checkErrors();
	void checkVelCartesian(const Pose&, const Pose&, double);
	void checkVelJoint(const sensor_msgs::JointState&, 
						const std::vector<double>&, double);
	
};

}


#endif /* DVRK_ARM_HPP_ */
