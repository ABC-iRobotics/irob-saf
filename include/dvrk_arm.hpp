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
#include "dvrk_arm_types.hpp"
#include "dvrk_arm_topics.hpp"
#include "vector_3d.hpp"
#include "quaternion.hpp"
#include "trajectory.hpp"


class DVRKArm {

public:
    // Constants
    static const std::string HOME_CMD;
    static const std::string HOME_DONE;
    static const std::string STATE_POSITION_JOINT;
    static const std::string STATE_POSITION_CARTESIAN;

private:
    const DVRKArmTypes arm_typ;
    ros::NodeHandle nh;

    // States
    std_msgs::String robot_state;
    sensor_msgs::JointState position_joint;
    geometry_msgs::PoseStamped position_cartesian_current;

    // Subscribers
    ros::Subscriber robot_state_sub;
    ros::Subscriber state_joint_current_sub;
    ros::Subscriber position_cartesian_current_sub;


    // Publishers
    ros::Publisher robot_state_pub;
    ros::Publisher position_joint_pub;
    ros::Publisher position_cartesian_pub;



public:
    DVRKArm(ros::NodeHandle, DVRKArmTypes);
	virtual ~DVRKArm();

    // Callbacks
    void robotStateCB(const std_msgs::String);
    void stateJointCurrentCB(const sensor_msgs::JointStateConstPtr&);
    void positionCartesianCurrentCB(const geometry_msgs::PoseStampedConstPtr&);

    bool subscribe(const DVRKArmTopics);
    bool advertise(const DVRKArmTopics);
    
    double getJointStateCurrent(int);
    Vector3D getPositionCartesianCurrent();
    Quaternion getOrientationCartesianCurrent();

    //DVRK actions
    bool home();
    bool setRobotState(std::string);
    void moveJointRelative(int, double );
    void moveJointAbsolute(int, double );
    void moveCartesianRelative(Vector3D);
    void moveCartesianAbsolute(Vector3D);
    void moveCartesianAbsolute(Quaternion);
    void moveCartesianAbsolute(Vector3D, Quaternion);

	void playTrajectory(Trajectory<Vector3D>&);
	void playTrajectory(Trajectory<Quaternion>&);
	void playTrajectory(Trajectory<Vector3D>&, Trajectory<Quaternion>&);
	void playTrajectory(int, Trajectory<double>&);
	
};

#endif /* DVRK_ARM_HPP_ */
