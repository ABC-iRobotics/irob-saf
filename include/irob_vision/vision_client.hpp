/*
 * 	vision_client.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-31
 *
 */

#ifndef VISION_CLIENT_HPP_
#define VISION_CLIENT_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "irob_utils/pose.hpp"

#include "irob_utils/utils.hpp"




namespace ias {

/* 
 * template<MsgT, DataT> 
 * DataT unwrapMsg(const MsgT& msg) should be implemented
 */
template <class MsgT, class DataT>
class VisionClient {

public:

   

protected:
    ros::NodeHandle nh;

   	// States
   	MsgT result_msg;
   	
    void subscribeTopics();

public:
	VisionClient(ros::NodeHandle);
	~VisionClient();

    // Callbacks 
    void resultCB(const MsgT::ConstPtr&);   
	
	// Does not block if the result is not valid
	DataT getResult();
};


template <class MsgT, class DataT>
VisionClient<MsgT, DataT>::VisionClient(ros::NodeHandle nh): nh(nh)
{
	subscribeTopics();
}


template <class MsgT, class DataT>
VisionClient<MsgT, DataT>::~VisionClient() {}



template <class MsgT, class DataT>
void VisionClient<MsgT>::subscribeTopics() 
{                 	            						
   	result_sub = nh.subscribe<MstgT>(
   					"result", 1000, 
   					&VisionClient<MsgT, DataT>::resultCB,this);
}

// Callbacks
template <class MsgT, class DataT>
void VisionClient<MsgT>::resultCB(const MsgT::ConstPtr& msg);
{
    result_msg = *msg;
}

/* 
 * template<MsgT, DataT> 
 * DataT unwrapMsg(const MsgT& msg) is used here
 */
template <class MsgT, class DataT>
DataT VisionServer<MsgT, DataT>::getResult()
{
	ros::spinOnce();
	
	return unwrapMsg(msg);
}






















}
#endif /* VISION_CLIENT_HPP_ */
