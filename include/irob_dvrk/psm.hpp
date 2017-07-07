/*
 * 	psm.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-17
 *
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
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include "irob_utils/utils.hpp"
#include "irob_dvrk/arm.hpp"
#include "irob_dvrk/arm_types.hpp"
#include "irob_dvrk/topics.hpp"
#include "irob_utils/pose.hpp"
#include "irob_utils/trajectory.hpp"

using namespace irob_autosurg;

namespace irob_dvrk {

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
