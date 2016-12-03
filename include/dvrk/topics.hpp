/*
 *  dvrk_topics.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *
 */

#ifndef DVRK_TOPICS_HPP_
#define DVRK_TOPICS_HPP_

#include <iostream>
#include "dvrk/arm_types.hpp"

namespace dvrk {

class Topics {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const Topics SET_ROBOT_STATE;
    static const Topics GET_ROBOT_STATE;
    static const Topics SET_POSITION_JOINT;
    static const Topics GET_STATE_JOINT_CURRENT;
    static const Topics SET_POSITION_CARTESIAN;
    static const Topics GET_POSITION_CARTESIAN_CURRENT;
    static const Topics SET_POSITION_JAW;
    
    static const Topics GET_ERROR;
    static const Topics GET_WARNING;


  private:
    static const std::string TOPIC_NAMESPACE;

    const std::string name;

  private:
    Topics( std::string name): name(name){ }

  public:
    bool operator==(const Topics& other) const{
        return other.name==this->name;
    }

    std::string getFullName(ArmTypes type)
    {
    	return "/" + TOPIC_NAMESPACE + "/" + type.name + "/" + name;
    }
};

}

#endif
