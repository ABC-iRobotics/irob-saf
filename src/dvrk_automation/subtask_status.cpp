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
// so the private SubtaskStatus constructor can be used.

const SubtaskStatus SubtaskStatus::NEW_DISSECTION_TARGET_NEEDED
                    = SubtaskStatus("new_dissection_target_needed");

const SubtaskStatus SubtaskStatus::WAITING_FOR_TARGET
                    = SubtaskStatus("waiting_for_target");
                    
const SubtaskStatus SubtaskStatus::NEW_DISTANT_TARGET_NEEDED
                    = SubtaskStatus("new_distant_target_needed");
                    
const SubtaskStatus SubtaskStatus::GOING_TO_TARGET
                    = SubtaskStatus("going_to_target");
                    
const SubtaskStatus SubtaskStatus::GOAL_REACHED
                    = SubtaskStatus("goal_reached");
                    
const SubtaskStatus SubtaskStatus::DP_REACHED
                    = SubtaskStatus("dp_reached");
                    
const SubtaskStatus SubtaskStatus::PERFORMING_DISSECTION
                    = SubtaskStatus("performing_dissection");
                 
const SubtaskStatus SubtaskStatus::ABORT
                    = SubtaskStatus("abort");
                    
}

