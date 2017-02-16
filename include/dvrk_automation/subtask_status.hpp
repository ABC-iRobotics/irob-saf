/*
 *  subtask_status.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-02-16
 *
 */

#ifndef SUBTASK_STATUS_HPP_
#define SUBTASK_STATUS_HPP_

#include <iostream>

namespace dvrk_automation {

class SubtaskStatus {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const SubtaskStatus NEW_DISSECTION_TARGET_NEEDED;
    static const SubtaskStatus NEW_DISTANT_TARGET_NEEDED;
    static const SubtaskStatus GOING_TO_TARGET;
    static const SubtaskStatus PERFORMING_DISSECTION;
    static const SubtaskStatus ABORT;

  private:
    const std::string command;

  private:
    Topics( std::string command): command(command){ }

  public:
    bool operator==(const SubtaskStatus& other) const{
        return other.command==this->command;
    }

    std::string getCommand()
    {
    	return command;
    }
};

}

#endif
