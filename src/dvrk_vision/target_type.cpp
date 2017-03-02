/*
 *  target_type.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-03-02
 *
 */

#include "dvrk_vision/target_type.hpp"

namespace dvrk_vision {

// Enum value DEFINITIONS
// The initialization occurs in the scope of the class,
// so the private SubtaskStatus constructor can be used.

const TargetType TargetType::GOAL
                    = TargetType("goal");

const TargetType TargetType::DP
                    = TargetType("dp");
                    
                    
}

