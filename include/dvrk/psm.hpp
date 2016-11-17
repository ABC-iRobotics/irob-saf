/*
 * psm.hpp
 *
 *  Created on: 2016. nov. 17.
 *      Author: tamas
 */

#ifndef DVRK_PSM_HPP_
#define DVRK_PSM_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "dvrk/arm_types.hpp"
#include "dvrk/topics.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include "dvrk/utils.hpp"
#include "dvrk/arm.hpp"

namespace dvrk {

class PSM: public Arm {


private:

    // Publishers
    ros::Publisher position_jaw_pub;
    bool advertise(const Topics);

public:
    PSM(ros::NodeHandle, ArmTypes, bool);
	~PSM();
    
    Pose getPoseCurrent();
    
    void moveCartesianAbsolute(Pose, double = 0.01);
    void moveJawRelative(double, double = 0.01);
    void moveJawAbsolute(double, double = 0.01);
	
};

}

#endif /* DVRK_PSM_HPP_ */
