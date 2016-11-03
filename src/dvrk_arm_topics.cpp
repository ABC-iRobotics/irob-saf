/*
 * dvrk_arm.cpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#include "dvrk_arm_topics.hpp"

// Enum value DEFINITIONS
// The initialization occurs in the scope of the class,
// so the private DVRKArmTopics constructor can be used.

const std::string DVRKArmTopics::TOPIC_NAMESPACE= "dvrk";

const DVRKArmTopics DVRKArmTopics::SET_ROBOT_STATE
                    = DVRKArmTopics("set_robot_state");

const DVRKArmTopics DVRKArmTopics::GET_ROBOT_STATE
                    = DVRKArmTopics("robot_state");

const DVRKArmTopics DVRKArmTopics::SET_POSITION_JOINT
                    = DVRKArmTopics("set_position_joint");
                    // = DVRKArmTopics("set_position_goal_joint");

const DVRKArmTopics DVRKArmTopics::GET_STATE_JOINT_CURRENT
                    = DVRKArmTopics("state_joint_current");

const DVRKArmTopics DVRKArmTopics::SET_POSITION_CARTESIAN
                    = DVRKArmTopics("set_position_cartesian");
                    //= DVRKArmTopics("set_position_goal_cartesian");

const DVRKArmTopics DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT
                    = DVRKArmTopics("position_cartesian_current");
                    
const DVRKArmTopics DVRKArmTopics::SET_POSITION_JAW
                    = DVRKArmTopics("set_jaw_position");

