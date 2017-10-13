/*
 * 	arm_types.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *
 */
 
#ifndef DVRK_ARM_PARAMS_
#define DVRK_ARM_PARAMS_

#include <iostream>
#include <string>
#include <vector>
#include <irob_utils/pose.hpp>

namespace ias {

class ArmTypes {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const ArmTypes MTML;
    static const ArmTypes MTMR;
    static const ArmTypes PSM1;
    static const ArmTypes PSM2;
    static const ArmTypes ECM;

	// Attributes
    const std::string name;
    const int dof;
    const Pose::Distance maxVelPose; // unit/sec
    const std::vector<double> maxVelJoint; // unit/sec

  private:
    ArmTypes( std::string name, int dof,
    			Pose::Distance maxVelPose, std::vector<double> maxVelJoint): 
    			name(name), dof(dof), 
    			maxVelPose(maxVelPose),
    			maxVelJoint(maxVelJoint) { }

  public:    
    static const ArmTypes typeForString(std::string name)
    {
    	if (name == MTML.name)
    		return MTML;
    	if (name == MTMR.name)
    		return MTMR;
    	if (name == PSM1.name)
    		return PSM1;
    	if (name == PSM2.name)
    		return PSM2;
    	if (name == ECM.name)
    		return ECM;
    	return PSM1;
    }
    
    bool operator==(const ArmTypes& other) const
    {
    	return name == other.name;
    }
};

}
#endif
