/*
 * 	retractor.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-09-05
 *
 */

#ifndef RETRACTOR_HPP_
#define RETRACTOR_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <geometry_msgs/Point.h>

#include <irob_utils/pose.hpp>
#include <irob_motion/maneuver_client.hpp>
#include <irob_vision_support/vision_client.hpp>


namespace ias {

class Retractor {


protected:
	std::vector<std::string> arm_names;
    ros::NodeHandle nh;
    
    ManeuverClient maneuver;
    VisionClient<geometry_msgs::Point, Eigen::Vector3d> vision;
    
   
    
public:
	Retractor(ros::NodeHandle, std::vector<std::string>);
	~Retractor();
	 void graspTissue();
	 void retract();
	
};

}
#endif /* RETRACTOR_HPP_ */
