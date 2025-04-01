/*
*  topic_name_loader.hpp
 *
 *  Author(s): Tamas Levendovics
 *  Created on: 2017-07-10
 *
 */

#ifndef TOPIC_NAME_LOADER_HPP_
#define TOPIC_NAME_LOADER_HPP_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
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
        static std::string load(std::shared_ptr<rclcpp::Node> nh, std::string topic_param_name)
        {
            std::string topic;
            if (nh->get_parameter(topic_param_name, topic)) {
                return "/" + topic;
            }
            return ""; // Return empty if parameter not found
        }

        /**
         * Load topic with topic name and extra identifier.
         */
        static std::string load(std::shared_ptr<rclcpp::Node> nh, std::string topic_middle, std::string topic_param_name)
        {
            std::string topic;
            if (nh->get_parameter(topic_param_name, topic)) {
                return "/" + topic_middle + "/" + topic;
            }
            return ""; // Return empty if parameter not found
        }

        /**
         * Load topic with topic name using nodehandle.
         */
        static std::string load(std::shared_ptr<rclcpp::Node> nh, std::string topic_param_name)
        {
            std::string topic;
            if (nh->get_parameter(topic_param_name, topic)) {
                return "/" + topic;
            }
            return ""; // Return empty if parameter not found
        }

        /**
         * Load topic with topic name and extra identifier using nodehandle.
         */
        static std::string load(std::shared_ptr<rclcpp::Node> nh, std::string topic_middle, std::string topic_param_name)
        {
            std::string topic;
            if (nh->get_parameter(topic_param_name, topic)) {
                return "/" + topic_middle + "/" + topic;
            }
            return ""; // Return empty if parameter not found
        }

    };

}

#endif
