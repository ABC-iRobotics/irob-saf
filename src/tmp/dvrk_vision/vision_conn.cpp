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
const std::string VisionConn::SERVICE_NAME_MOVEMENT_TARGET
                    = "movement_target";
const std::string VisionConn::SERVICE_NAME_DO_TASK
                    = "do_task";
const std::string VisionConn::TOPIC_NAME_ERR
                    = "error";
const std::string VisionConn::TOPIC_NAME_SUBTASK_STATUS
                    = "subtask_status";

VisionConn::VisionConn(
			ros::NodeHandle nh, std::string registration_file,
			std::string topic_suffix): nh(nh), topic_suffix(topic_suffix)
{
	loadRegistration(registration_file);
	
	// Subscribe and advertise topics
	subscribe(SERVICE_NAME_MOVEMENT_TARGET);
	subscribe(SERVICE_NAME_DO_TASK);
	subscribe(TOPIC_NAME_ERR);
	
	advertise(TOPIC_NAME_SUBTASK_STATUS);
}

VisionConn::~VisionConn()
{
	// TODO Auto-generated destructor stub
}

void VisionConn::loadRegistration(std::string registration_file)
{
	std::ifstream cfgfile(registration_file.c_str());
    if (!cfgfile.is_open())
    	throw std::runtime_error("Cannot open file " + registration_file);
    if (cfgfile.eof())
    	throw std::runtime_error("Cfgfile " + registration_file + " is empty.");
   	
   	double x, y, z;
   	
    cfgfile >> x >> std::ws >> y >> std::ws >> z >> std::ws;
    t << x, y, z;
    
    for (int i = 0; i < 3; i++)
    {
    	cfgfile >> x >> std::ws >> y >> std::ws >> z >> std::ws;
    	R(i,0) = x;
    	R(i,1) = y;
    	R(i,2) = z;
    }
    
    cfgfile.close();
    
    ROS_INFO_STREAM("Registration read: "<< std::endl << t << std::endl << R);
}

/*
 * Callbacks
 */
void VisionConn::errorCB(const std_msgs::String msg) 
{
    error  = msg;
    //ROS_INFO_STREAM(msg);
}


bool VisionConn::subscribe(std::string topic) 
{
	std::string topic_name_full =  "/" + TOPIC_NAMESPACE + "/" + topic + 
		"_" + topic_suffix;
	
	//ROS_INFO_STREAM(topic_name_full);
	
	if(topic == SERVICE_NAME_MOVEMENT_TARGET)
	{
    	movement_target_cli = nh.serviceClient
    	<irob_dvrk_automation::TargetPose>(topic_name_full, true);
    }
    else if(topic == SERVICE_NAME_DO_TASK)
	{
    	task_done_cli = nh.serviceClient<irob_dvrk_automation::BoolQuery>(
               			topic_name_full, true);
    }
    else if(topic == TOPIC_NAME_ERR)
	{
    	error_sub = nh.subscribe<std_msgs::String>(
               			topic_name_full, 1000,
                        &VisionConn::errorCB,this);
    }
    else 
    {
         ROS_WARN_STREAM("Subscribing to invalid topic " << topic_name_full);
         return false;
    }
    
    ROS_INFO_STREAM("Subscribed to topic " << topic);
    return true;
}


bool VisionConn::advertise(std::string topic) 
{
	std::string topic_name_full =  "/" + TOPIC_NAMESPACE + "/" + topic
		+ "_" + topic_suffix;	
	
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


void VisionConn::getTargetCurrent(TargetType target_type,
									dvrk::Pose& movement_target,
								 	PositionType& position_type)
{
	// Call service
	irob_dvrk_automation::TargetPose req;
	req.request.target_type = target_type.getCommand();
	movement_target_cli.call(req);
	
	// Transform coordinates
	dvrk::Pose cam_target(req.response.pose, 0);
    cam_target = cam_target.rotate(R);
    cam_target += t;
 	movement_target  = cam_target;
 	
 	position_type = PositionType::fromCmd(req.response.position_type);
    
    ros::spinOnce();
	checkErrors();
}


void VisionConn::sendSubtaskStatus(std::string status)
{
        std_msgs::String msg;
        std::stringstream ss;
        ss << status;
        msg.data = ss.str();
        subtask_status_pub.publish(msg);
        ros::spinOnce();
        checkErrors();
}


bool VisionConn::needToDoTask()
{
	// Call service
	irob_dvrk_automation::BoolQuery req;
	task_done_cli.call(req);
	
	ros::spinOnce();
	checkErrors();	
 	return req.response.data;
}

void VisionConn::checkErrors()
{    
	ros::spinOnce();
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
}


}
