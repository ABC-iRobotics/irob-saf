/*
 *  subtask_status.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-02-16
 *
 */

#include "dvrk_automation/subtask_status.hpp"

namespace dvrk_automation {

// Enum value DEFINITIONS
// The initialization occurs in the scope of the class,
// so the private Topics constructor can be used.

const Topics SubtaskStatus::NEW_DISSECTION_TARGET_NEEDED
                    = SubtaskStatus("new_dissection_target_needed");

const Topics SubtaskStatus::NEW_DISTANT_TARGET_NEEDED
                    = SubtaskStatus("new_distant_target_needed");
                    
const Topics SubtaskStatus::GOING_TO_TARGET
                    = SubtaskStatus("going_to_target");
                    
const Topics SubtaskStatus::PERFORMING_DISSECTION
                    = SubtaskStatus("set_robot_state");
                 
const Topics SubtaskStatus::ABORT
                    = SubtaskStatus("abort");
                    
}

