/*
 *  position_type.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-05-15
 *
 */

#ifndef POSITION_TYPE_HPP_
#define POSITION_TYPE_HPP_

#include <iostream>
#include "irob_dvrk_automation/TargetPose.h"

namespace dvrk_vision {

class PositionType {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const PositionType GOAL;
    static const PositionType DP;


  private:
    int command;

  private:
    PositionType(int command): command(command){ }

  public:
  	PositionType(const PositionType& other): command(other.command){ }
  	
  	void operator=(const PositionType& other)
    {
        command=other.command;
    }
  
    bool operator==(const PositionType& other) const
    {
        return other.command==this->command;
    }

    int getCommand() const
    {
    	return command;
    }
    
    static PositionType fromCmd(int cmd)
    {
    	if (cmd == GOAL.command)
    		return GOAL;
    	return DP;
    }
};

}

#endif
