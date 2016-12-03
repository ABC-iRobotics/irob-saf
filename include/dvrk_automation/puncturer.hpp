/*
 * 	puncturer.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-03
 *
 */

#ifndef DVRK_PUNCTURER_HPP_
#define DVRK_PUNCTURER_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include "dvrk/arm_types.hpp"
#include "dvrk/topics.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory.hpp"
#include "dvrk/utils.hpp"
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "optoforce/optoforce_listener.hpp"


namespace dvrk_automation {

class Puncturer {
   

private:
    ros::NodeHandle nh;

    // States
    dvrk::PSM psm;
    optoforce::OptoforceListener oforce;

    
public:
	Puncturer(ros::NodeHandle, dvrk::ArmTypes);
	~Puncturer();

	/*
	void checkErrors();
	void checkVelCartesian(const Pose&, const Pose&, double);
	void checkVelJoint(const sensor_msgs::JointState&, 
						const std::vector<double>&, double);
	*/
	
};

}


#endif /* DVRK_PUNCTURER_HPP_ */
