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

namespace dvrk_vision {

class TargetType {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const TargetType GOAL;
    static const TargetType DP;


  private:
    std::string command;

  private:
    TargetType( std::string command): command(command){ }

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

    std::string getCommand() const
    {
    	return command;
    }
    
    static TargetType fromString(std::string cmd)
    {
    	if (cmd == GOAL.command)
    		return GOAL;
    	return DP;
    }
};

}

#endif
