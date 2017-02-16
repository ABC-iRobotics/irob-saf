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
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>

#include "dvrk/pose.hpp"

#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

namespace dvrk_vision {

class VisionConn {

public:
    // Constants
    static const std::string TOPIC_NAMESPACE;
    static const std::string TOPIC_NAME_MOVEMENT_TARGET;	// vision -> control
    static const std::string TOPIC_NAME_TARGET_VALID;		// vision -> control
    static const std::string TOPIC_NAME_SUBTASK_STATUS;		// control -> vision
    static const std::string TOPIC_NAME_TASK_DONE;			// vision -> control
    static const std::string TOPIC_NAME_ERR;			  // vision -> control    

private:
    ros::NodeHandle nh;

    // States
    dvrk::Pose movement_target;
    bool target_valid;
    bool task_done;
    std_msgs::String error;

    // Subscribers
    ros::Subscriber movement_target_sub;
    ros::Subscriber target_valid_sub;
    ros::Subscriber task_done_sub;
    ros::Subscriber error_sub;

    // Publishers
    ros::Publisher subtask_status_pub;

    bool subscribe(const std::string);
    bool advertise(const std::string);

public:
	VisionConn(ros::NodeHandle);
	~VisionConn();

    // Callbacks
    void movementTargetCB(const geometry_msgs::PoseConstPtr&);
    void targetValidCB(const std_msgs::Bool);  
    void taskDoneCB(const std_msgs::Bool); 
    void errorCB(const std_msgs::String); 
    
    dvrk::Pose getTargetCurrent();
    bool isTargetValid();
    bool isTaskDone();
    void checkErrors();	
};

}


#endif /* VISION_CONN_HPP_ */
