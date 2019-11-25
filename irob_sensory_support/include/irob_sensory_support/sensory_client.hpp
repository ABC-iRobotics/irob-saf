/*
 * 	sensory_client.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-31
 *
 *  Generic client to receive computer sensory information
 *  of any datatype. Designed to be used as member of
 *  subtask-level logic node.
 *
 */

#ifndef SENSORY_CLIENT_HPP_
#define SENSORY_CLIENT_HPP_

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




namespace saf {

/* 
 * template<MsgT, DataT>
 * DataT unwrapMsg(const MsgT& msg) should be implemented
 */
template <class MsgT, class DataT>
class SensoryClient {

public:



protected:
  ros::NodeHandle nh;

  std::string topic_name;

  // States
  MsgT result_msg;

  ros::Subscriber result_sub;

  void subscribeTopics();

public:
  SensoryClient(ros::NodeHandle, std::string);
  ~SensoryClient();

  // Callbacks
  void resultCB(const typename MsgT::ConstPtr&);

  // Does not block if the result is not valid
  DataT getResult();
};


template <class MsgT, class DataT>
SensoryClient<MsgT, DataT>::SensoryClient(ros::NodeHandle nh, std::string topic_name): nh(nh), topic_name(topic_name)
{
  result_msg = makeNaN<MsgT>();
  subscribeTopics();
}


template <class MsgT, class DataT>
SensoryClient<MsgT, DataT>::~SensoryClient() {}



template <class MsgT, class DataT>
void SensoryClient<MsgT, DataT>::subscribeTopics()
{                 	            						
  result_sub = nh.subscribe<MsgT>(
        topic_name, 1000,
        &SensoryClient<MsgT, DataT>::resultCB,this);
}

// Callbacks
template <class MsgT, class DataT>
void SensoryClient<MsgT, DataT>::resultCB(const typename MsgT::ConstPtr& msg)
{
  result_msg = *msg;
}

/* 
 * template<MsgT, DataT>
 * DataT unwrapMsg(const MsgT& msg) is used here
 */
template <class MsgT, class DataT>
DataT SensoryClient<MsgT, DataT>::getResult()
{
  ros::spinOnce();

  MsgT tmp_msg = result_msg;
  result_msg = makeNaN<MsgT>();
  return unwrapMsg<MsgT, DataT>(tmp_msg);
}






















}
#endif /* VISION_CLIENT_HPP_ */
