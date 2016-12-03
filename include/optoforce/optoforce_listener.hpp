/*
 * 	optoforce_listener.hpp
 *
 * 	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-03
 *
 */

#ifndef OPTOFORCE_LISTENER_HPP_
#define OPTOFORCE_LISTENER_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <optoforcesensor/FT.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>

namespace optoforce {

class OptoforceListener {

public:
    // Constants
    static const std::string TOPIC_NAME;   

private:
    ros::NodeHandle nh;

    // States
    Eigen::Vector3d forces;

    // Subscribers
    ros::Subscriber forces_sub;

    bool subscribe(const std::string);

public:
	OptoforceListener(ros::NodeHandle);
	~OptoforceListener();

    // Callbacks
    void forcesCB(const optoforcesensor::FT); 
    
    Eigen::Vector3d getForcesCurrent();
	
};

}


#endif /* OPTOFORCE_LISTENER_HPP_ */
