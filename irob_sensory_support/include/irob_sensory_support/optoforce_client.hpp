/*
 * 	optoforce_client.hpp
 *
 * 	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-03
 *
 */

#ifndef OPTOFORCE_CLIENT_HPP_
#define OPTOFORCE_CLIENT_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

#include <optoforcesensor/FT.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>

namespace saf {

class OptoforceClient {

public:
    // Constants
    static const std::string TOPIC_NAME;   

private:
    ros::NodeHandle nh;

    // States
    Eigen::Vector3d forces;
    Eigen::Vector3d offsets;

    // Subscribers
    ros::Subscriber forces_sub;

    bool subscribe(const std::string);

public:
    OptoforceClient(ros::NodeHandle);
    ~OptoforceClient();

    // Callbacks
    void forcesCB(const optoforcesensor::FT); 
    
    Eigen::Vector3d getForcesCurrent();
    void calibrateOffsets();
	
};

}


#endif /* OPTOFORCE_LISTENER_HPP_ */
