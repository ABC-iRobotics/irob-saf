/*
 *  dvrk_arm.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *
 */

#include "dvrk/topics.hpp"

namespace dvrk {

// Enum value DEFINITIONS
// The initialization occurs in the scope of the class,
// so the private Topics constructor can be used.

const std::string Topics::TOPIC_NAMESPACE= "dvrk";

const Topics Topics::SET_ROBOT_STATE
                    = Topics("set_robot_state");

const Topics Topics::GET_ROBOT_STATE
                    = Topics("robot_state");

const Topics Topics::SET_POSITION_JOINT
                    = Topics("set_position_joint");
                    // = Topics("set_position_goal_joint");

const Topics Topics::GET_STATE_JOINT_CURRENT
                    = Topics("state_joint_current");

const Topics Topics::SET_POSITION_CARTESIAN
                    = Topics("set_position_cartesian");
                    //= Topics("set_position_goal_cartesian");

const Topics Topics::GET_POSITION_CARTESIAN_CURRENT
                    = Topics("position_cartesian_current");
                    
const Topics Topics::SET_POSITION_JAW
                    = Topics("set_jaw_position");
                    
const Topics Topics::GET_ERROR
                    = Topics("error");
                    
const Topics Topics::GET_WARNING
                    = Topics("warning");
                    
}

