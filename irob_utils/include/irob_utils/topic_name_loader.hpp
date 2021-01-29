/*
 *  topic_name_loader.hpp
 *
 *  Author(s): Tamas D. Nagy
 *	Created on: 2017-07-10
 *
 *
 */

#ifndef TOPIC_NAME_LOADER_HPP_
#define TOPIC_NAME_LOADER_HPP_

#include <iostream>
#include <ros/ros.h>
#include <string>


namespace saf {

class TopicNameLoader
{

private:
  TopicNameLoader() {}
public:

  /**
     * Load topic with topic name.
     */
  static std::string load(std::string topic_param_name)
  {

    std::string topic;
    ros::param::get(topic_param_name, topic);

    return "/" + topic;
  }

  /**
     * Load topic with topic name and extra identifier.
     */
  static std::string load(std::string topic_middle, std::string topic_param_name)
  {

    std::string topic;
    ros::param::get(topic_param_name, topic);

    return "/" + topic_middle + "/" + topic;
  }

  /**
     * Load topic with topic name using
     * nodehandle.
     */
  static std::string load(ros::NodeHandle nh,
                          std::string topic_param_name)
  {

    std::string topic;
    nh.getParam(topic_param_name, topic);

    return "/" + topic;
  }

  /**
     *  Load topic with topic name and extra identifier using
     * 	nodehandle.
     */
  static std::string load(ros::NodeHandle nh,
                          std::string topic_middle,
                          std::string topic_param_name)
  {

    std::string topic;
    nh.getParam(topic_param_name, topic);
    return "/" + topic_middle + "/" + topic;
  }


};

}

#endif
