/*
 *  target_type.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-03-02
 *
 */

#ifndef TARGET_TYPE_HPP_
#define TARGET_TYPE_HPP_

#include <iostream>
#include "irob_dvrk_automation/TargetPose.h"

namespace dvrk_vision {

class TargetType {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const TargetType DISSECTION;
    static const TargetType DISTANT;
    static const TargetType GRABBING;
    static const TargetType RETRACTION;


  private:
    int command;

  private:
    TargetType( int command): command(command){ }

  public:
  	TargetType(const TargetType& other): command(other.command){ }
  	
  	void operator=(const TargetType& other)
    {
        command=other.command;
    }
  
    bool operator==(const TargetType& other) const
    {
        return other.command==this->command;
    }

    int getCommand() const
    {
    	return command;
    }
    
    static TargetType fromCmd(int cmd)
    {
    	if (cmd == DISSECTION.command)
    		return DISSECTION;
    	if (cmd == DISTANT.command)
    		return DISTANT;	
    	if (cmd == GRABBING.command)
    		return GRABBING;	
    	if (cmd == RETRACTION.command)
    		return RETRACTION;	
    	return DISTANT;
    }
};

}

#endif
