/*
 * 	vision_conn.hpp
 *
 * 	Author(s): Tamas D. Nagy
 *	Created on: 2017-02-15
 *
 */

#ifndef VISION_CONN_HPP_
#define VISION_CONN_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <string>
#include <stdexcept>
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>

#include "dvrk/pose.hpp"
#include "dvrk_vision/target_type.hpp"
#include "dvrk_vision/position_type.hpp"

#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "irob_dvrk_automation/BoolQuery.h"
#include "irob_dvrk_automation/TargetPose.h"



namespace dvrk_vision {

class VisionConn {

public:
    // Constants
    static const std::string TOPIC_NAMESPACE;
    static const std::string SERVICE_NAME_MOVEMENT_TARGET;	// vision -> control
    static const std::string SERVICE_NAME_DO_TASK;		// vision -> control
    static const std::string TOPIC_NAME_ERR;			 // vision -> control   
    static const std::string TOPIC_NAME_SUBTASK_STATUS;		// control -> vision 

private:
    ros::NodeHandle nh;

    // States
    std_msgs::String error;
    
    // translation and rotation for registration
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    
    std::string topic_suffix;

   	// Clients
    ros::ServiceClient movement_target_cli;
    ros::ServiceClient task_done_cli;
    
    // Subscribers
    ros::Subscriber error_sub;
   
    // Publishers
    ros::Publisher subtask_status_pub;

    bool subscribe(const std::string);
    bool advertise(const std::string);

public:
	VisionConn(ros::NodeHandle, std::string, std::string);
	~VisionConn();
		
	void loadRegistration(std::string);
	
	void sendSubtaskStatus(std::string);
	
    // Callbacks
    void errorCB(const std_msgs::String); 
       
    void getTargetCurrent(TargetType, dvrk::Pose&, PositionType&);
    bool needToDoTask();
    void checkErrors();	
};

}


#endif /* VISION_CONN_HPP_ */
