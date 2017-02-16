/*
 * 	vision_conn.cpp
 *
 * 	Author(s): Tamas D. Nagy
 *	Created on: 2017-02-15
 *
 */

#include "dvrk_vision/vision_conn.hpp"
#include <numeric>
#include <chrono>

namespace dvrk_vision {

// TODO check topic name
const std::string VisionConn::TOPIC_NAMESPACE
                    = "dvrk_vision";
const std::string VisionConn::TOPIC_NAME_MOVEMENT_TARGET
                    = "movement_target";
const std::string VisionConn::TOPIC_NAME_TARGET_VALID
                    = "target_valid";
const std::string VisionConn::TOPIC_NAME_SUBTASK_STATUS
                    = "subtask_status";
const std::string VisionConn::TOPIC_NAME_TASK_DONE
                    = "task_done";
const std::string VisionConn::TOPIC_NAME_ERR
                    = "error";

VisionConn::VisionConn(
			ros::NodeHandle nh): nh(nh)
{
	// Subscribe and advertise topics
	subscribe("/" + TOPIC_NAMESPACE + "/" + TOPIC_NAME_MOVEMENT_TARGET);
	subscribe("/" + TOPIC_NAMESPACE + "/" + TOPIC_NAME_TARGET_VALID);
	subscribe("/" + TOPIC_NAMESPACE + "/" + TOPIC_NAME_TASK_DONE);
	subscribe("/" + TOPIC_NAMESPACE + "/" + TOPIC_NAME_ERR);
	
	advertise("/" + TOPIC_NAMESPACE + "/" + TOPIC_NAME_SUBTASK_STATUS);
      
}

VisionConn::~VisionConn()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void VisionConn::movementTargetCB(const geometry_msgs::PoseConstPtr& msg) 
{
    movement_target  = dvrk::Pose(*msg, 0);
    ROS_INFO_STREAM(movement_target);
}

void VisionConn::targetValidCB(const std_msgs::Bool msg) 
{
    target_valid  = msg.data;
    ROS_INFO_STREAM(task_done);
}

void VisionConn::taskDoneCB(const std_msgs::Bool msg) 
{
    task_done  = msg.data;
    ROS_INFO_STREAM(task_done);
}

void VisionConn::errorCB(const std_msgs::String msg) 
{
    error  = msg;
    ROS_INFO_STREAM(msg);
}


bool VisionConn::subscribe(std::string topic) 
{
	std::string topic_name_full =  "/" + TOPIC_NAMESPACE + "/" + topic;
	
	if(topic == TOPIC_NAME_MOVEMENT_TARGET)
	{
    	movement_target_sub = nh.subscribe<geometry_msgs::Pose>(
               			topic_name_full, 1000,
                        &VisionConn::movementTargetCB,this);
    }
    else if(topic == TOPIC_NAME_TARGET_VALID)
	{
    	target_valid_sub = nh.subscribe<std_msgs::Bool>(
               			topic_name_full, 1000,
                        &VisionConn::targetValidCB,this);
    }
    else if(topic == TOPIC_NAME_TASK_DONE)
	{
    	task_done_sub = nh.subscribe<std_msgs::Bool>(
               			topic_name_full, 1000,
                        &VisionConn::taskDoneCB,this);
    }
    else if(topic == TOPIC_NAME_ERR)
	{
    	error_sub = nh.subscribe<std_msgs::String>(
               			topic_name_full, 1000,
                        &VisionConn::errorCB,this);
    }
    else 
    {
         ROS_WARN_STREAM("Advertising invalid topic " << topic_name_full);
         return false;
    }
    
    ROS_INFO_STREAM("Subscribed to topic " << topic);
    return true;
}

bool VisionConn::advertise(std::string topic) 
{
	std::string topic_name_full =  "/" + TOPIC_NAMESPACE + "/" + topic;	
	
    if(topic == TOPIC_NAME_SUBTASK_STATUS)
    {
        subtask_status_pub = nh.advertise<std_msgs::String>(
                                   topic_name_full, 1000);
    }
    else 
    {
         ROS_WARN_STREAM("Advertising invalid topic " << topic_name_full);
         return false;
    }
    ROS_DEBUG_STREAM("Advertised topic " << topic_name_full);
    return true;
}


dvrk::Pose VisionConn::getTargetCurrent()
{
	ros::spinOnce();
 	return movement_target;
}

bool VisionConn::isTargetValid()
{
	ros::spinOnce();
 	return target_valid;
}

bool VisionConn::isTaskDone()
{
	ros::spinOnce();	
 	return task_done;
}

void VisionConn::checkErrors()
{    
	ros::spinOnce();
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
}


}
