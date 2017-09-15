/*
 *  topic_name_loader.hpp
 *	
 *  Author(s): Tamas D. Nagy
 *	Created on: 2017-07-10
 *  
 */

#ifndef TOPIC_NAME_LOADER_HPP_
#define TOPIC_NAME_LOADER_HPP_

#include <iostream>
#include <ros/ros.h>
#include <string>


namespace ias {

class TopicNameLoader
{

	private:
		TopicNameLoader() {}
	public:
		
		/**
		 * Load topic with namespace and topic name.
		 */
		static std::string load(std::string namespace_param_name, 
			std::string topic_param_name)
		{
			std::string ns;
			ros::param::get(namespace_param_name, ns);
			
			std::string topic;
			ros::param::get(topic_param_name, topic);
			
			return "/" + ns + "/" + topic;
		}
		
		/**
		 * Load topic with namespace, topic name and extra identifier.
		 */
		static std::string load(std::string namespace_param_name, 
			std::string topic_middle, std::string topic_param_name)
		{
			std::string ns;
			ros::param::get(namespace_param_name, ns);
			
			std::string topic;
			ros::param::get(topic_param_name, topic);
			
			return "/" + ns + "/" + topic_middle + "/" + topic;
		}
		
		/**
		 * Load topic with namespace and topic name using
		 * nodehandle.
		 */
		static std::string load(ros::NodeHandle nh,
			std::string namespace_param_name, 
			std::string topic_param_name)
		{
			std::string ns;
			nh.getParam(namespace_param_name, ns);
			
			std::string topic;
			nh.getParam(topic_param_name, topic);
			
			return "/" + ns + "/" + topic;
		}
		
		/**
		 *  Load topic with namespace, topic name and extra identifier using
		 * 	nodehandle.
		 */
		static std::string load(ros::NodeHandle nh,
			std::string namespace_param_name,
			std::string topic_middle, 
			std::string topic_param_name)
		{
			std::string ns;
			nh.getParam(namespace_param_name, ns);
			
			std::string topic;
			nh.getParam(topic_param_name, topic);
			return "/" + ns + "/" + topic_middle + "/" + topic;
		}
	

};

}

#endif
