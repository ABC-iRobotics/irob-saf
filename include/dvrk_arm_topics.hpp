/*
 * dvrk_arm_topics.hpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#ifndef DVRK_ARM_TOPICS_HPP_
#define DVRK_ARM_TOPICS_HPP_

#include <iostream>
#include "dvrk_arm_types.hpp"

class DVRKArmTopics {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const DVRKArmTopics SET_ROBOT_STATE;
    static const DVRKArmTopics GET_ROBOT_STATE;
    static const DVRKArmTopics SET_POSITION_JOINT;
    static const DVRKArmTopics GET_STATE_JOINT_CURRENT;
    static const DVRKArmTopics SET_POSITION_CARTESIAN;
    static const DVRKArmTopics GET_POSITION_CARTESIAN_CURRENT;
    static const DVRKArmTopics SET_POSITION_JAW;
    
    static const DVRKArmTopics GET_ERROR;
    static const DVRKArmTopics GET_WARNING;


  private:
    static const std::string TOPIC_NAMESPACE;

    const std::string name;

  private:
    DVRKArmTopics( std::string name): name(name){ }

  public:
    bool operator==(const DVRKArmTopics& other) const{
        return other.name==this->name;
    }

    std::string getFullName(DVRKArmTypes type)
    {
    	return "/" + TOPIC_NAMESPACE + "/" + type.name + "/" + name;
    }
};


#endif
