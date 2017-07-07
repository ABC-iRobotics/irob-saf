/*
 *  position_type.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-05-15
 *
 */

#include "dvrk_vision/position_type.hpp"

namespace dvrk_vision {

// Enum value DEFINITIONS
// The initialization occurs in the scope of the class,
// so the private SubtaskStatus constructor can be used.

const PositionType PositionType::GOAL
                    = PositionType(
                    irob_dvrk_automation::TargetPose::Response::GOAL);

const PositionType PositionType::DP
                    = PositionType(
                    irob_dvrk_automation::TargetPose::Response::DP);
                    
                    
}

