/*
 * 	vision_client.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-31
 *
 *  Generic client to receive computer vision information
 *  of any datatype. Designed to be used as member of
 *  subtask-level logic node.
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
#include "irob_utils/tool_pose.hpp"

#include "irob_utils/utils.hpp"




namespace saf {

/* 
 * template<MsgT, DataT>
 * DataT unwrapMsg(const MsgT& msg) should be implemented
 */
template <class MsgT, class DataT>
class VisionClient {

public:



protected:
  ros::NodeHandle nh;

  std::string topic_name;

  // States
  MsgT result_msg;

  ros::Subscriber result_sub;

  void subscribeTopics();

public:
  VisionClient(ros::NodeHandle, std::string);
  ~VisionClient();

  // Callbacks
  void resultCB(const typename MsgT::ConstPtr&);

  // Does not block if the result is not valid
  DataT getResult();
};


template <class MsgT, class DataT>
VisionClient<MsgT, DataT>::VisionClient(ros::NodeHandle nh, std::string topic_name): nh(nh), topic_name(topic_name)
{
  result_msg = makeNaN<MsgT>();
  subscribeTopics();
}


template <class MsgT, class DataT>
VisionClient<MsgT, DataT>::~VisionClient() {}



template <class MsgT, class DataT>
void VisionClient<MsgT, DataT>::subscribeTopics() 
{                 	            						
  result_sub = nh.subscribe<MsgT>(
        topic_name, 1000,
        &VisionClient<MsgT, DataT>::resultCB,this);
}

// Callbacks
template <class MsgT, class DataT>
void VisionClient<MsgT, DataT>::resultCB(const typename MsgT::ConstPtr& msg)
{
  result_msg = *msg;
}

/* 
 * template<MsgT, DataT>
 * DataT unwrapMsg(const MsgT& msg) is used here
 */
template <class MsgT, class DataT>
DataT VisionClient<MsgT, DataT>::getResult()
{
  ros::spinOnce();

  MsgT tmp_msg = result_msg;
  result_msg = makeNaN<MsgT>();
  return unwrapMsg<MsgT, DataT>(tmp_msg);
}






















}
#endif /* VISION_CLIENT_HPP_ */
