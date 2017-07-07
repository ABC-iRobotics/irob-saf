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

const TargetType TargetType::DISSECTION
                    = TargetType(
                    irob_dvrk_automation::TargetPose::Request::DISSECTION);

const TargetType TargetType::DISTANT
                    = TargetType(
                    irob_dvrk_automation::TargetPose::Request::DISTANT);
                    
const TargetType TargetType::GRABBING
                    = TargetType(
                    irob_dvrk_automation::TargetPose::Request::GRABBING);
                    
const TargetType TargetType::RETRACTION
                    = TargetType(
                    irob_dvrk_automation::TargetPose::Request::RETRACTION);
                    
                    
}

