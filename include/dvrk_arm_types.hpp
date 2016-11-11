#ifndef DVRK_ARM_PARAMS_
#define DVRK_ARM_PARAMS_

#include <iostream>
#include <string>
#include <vector>
#include "pose.hpp"

class DVRKArmTypes {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const DVRKArmTypes MTML;
    static const DVRKArmTypes MTMR;
    static const DVRKArmTypes PSM1;
    static const DVRKArmTypes PSM2;
    static const DVRKArmTypes ECM;

	// Attributes
    const std::string name;
    const int dof;
    const Pose::Distance maxVelPose; // unit/sec
    const std::vector<double> maxVelJoint; // unit/sec

  private:
    DVRKArmTypes( std::string name, int dof,
    			Pose::Distance maxVelPose, std::vector<double> maxVelJoint): 
    			name(name), dof(dof), 
    			maxVelPose(maxVelPose),
    			maxVelJoint(maxVelJoint) { }

  public:    
    static const DVRKArmTypes typeForString(std::string name)
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
    
    bool operator==(const DVRKArmTypes& other) const
    {
    	return name == other.name;
    }
};

#endif
