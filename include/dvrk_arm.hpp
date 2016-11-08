/*
 * dvrk_arm.hpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
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
#include "dvrk_arm_types.hpp"
#include "dvrk_arm_topics.hpp"
#include "pose.hpp"
#include "trajectory.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <math.h>
#include "dvrk_utils.hpp"



class DVRKArm {

public:
    // Constants
    static const std::string HOME_CMD;
    static const std::string HOME_DONE;
    static const std::string STATE_POSITION_JOINT;
    static const std::string STATE_POSITION_CARTESIAN;
    static const bool ACTIVE = true;
    static const bool PASSIVE = false;
    

private:
    const DVRKArmTypes arm_typ;
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
    ros::Publisher position_jaw_pub;
    
    bool subscribe(const DVRKArmTopics);
    bool advertise(const DVRKArmTopics);

public:
    DVRKArm(ros::NodeHandle, DVRKArmTypes, bool);
	virtual ~DVRKArm();

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
    Pose getPoseCurrent();

    //DVRK actions
    bool home();
    bool setRobotState(std::string);
    void moveJointRelative(int, double );
    void moveJointAbsolute(int, double );
    void moveCartesianRelative(Eigen::Vector3d);
    void moveCartesianAbsolute(Eigen::Vector3d);
    void moveCartesianAbsolute(Eigen::Quaternion<double>);
    void moveCartesianAbsolute(Pose);
    
    void moveJawRelative(double);
    void moveJawAbsolute(double);

	void playTrajectory(Trajectory<Eigen::Vector3d>&);
	void playTrajectory(Trajectory<Eigen::Quaternion<double>>&);
	void playTrajectory(Trajectory<Pose>&);
	void playTrajectory(int, Trajectory<double>&);
	
	void recordTrajectory(Trajectory<Eigen::Vector3d>&);
	void recordTrajectory(Trajectory<Pose>&);
	
};


#endif /* DVRK_ARM_HPP_ */
