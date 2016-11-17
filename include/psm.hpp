/*
 * psm.hpp
 *
 *  Created on: 2016. nov. 17.
 *      Author: tamas
 */

#ifndef PSM_HPP_
#define PSM_HPP_

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
#include <cmath>
#include "dvrk_utils.hpp"
#include "dvrk_arm.hpp"



class PSM: public DVRKArm {


private:

    // Publishers
    ros::Publisher position_jaw_pub;
    bool advertise(const DVRKArmTopics);

public:
    PSM(ros::NodeHandle, DVRKArmTypes, bool);
	virtual ~PSM();
    
    Pose getPoseCurrent();
    
    void moveCartesianAbsolute(Pose, double = 0.01);
    void moveJawRelative(double, double = 0.01);
    void moveJawAbsolute(double, double = 0.01);
	
};


#endif /* PSM_HPP_ */
