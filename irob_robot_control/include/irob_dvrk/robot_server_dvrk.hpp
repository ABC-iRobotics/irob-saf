/*
 * 	robot_server_dvrk.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-07
 *
 */

#ifndef ROBOT_SERVER_DVRK_HPP_
#define ROBOT_SERVER_DVRK_HPP_

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
#include <irob_dvrk/arm_types.hpp>
#include <irob_general_robot/robot_server.hpp>

namespace ias {

class RobotServerDVRK: public RobotServer {

public:

    // Constants
    static const std::string HOME_CMD;
    static const std::string HOME_DONE;
    static const std::string STATE_POSITION_JOINT;
    static const std::string STATE_POSITION_CARTESIAN;
   

protected:
    const ArmTypes arm_typ;


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
    
    void subscribeLowLevelTopics();
    void advertiseLowLevelTopics();

public:
	RobotServerDVRK(ros::NodeHandle, ArmTypes, std::string, std::string, bool);
	~RobotServerDVRK();

    // Callbacks
    void initArm(bool);
    void resetPose(bool);
    void stop();
    void followTrajectory(Trajectory<Pose>);
    
    void robotStateCB(const std_msgs::String);
    void errorCB(const std_msgs::String);
    void warningCB(const std_msgs::String);
    void stateJointCurrentCB(const sensor_msgs::JointStateConstPtr&);
    virtual void positionCartesianCurrentCB(
    	const geometry_msgs::PoseStampedConstPtr&);

   
    void loadRegistration(std::string);
    
    double getJointStateCurrent(int);
    std::vector<double> getJointStateCurrent();
    Eigen::Vector3d getPositionCartesianCurrent();
    Eigen::Quaternion<double> getOrientationCartesianCurrent();
    Pose getPoseCurrent();

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
#endif /* ROBOT_SERVER_DVRK_HPP_ */
