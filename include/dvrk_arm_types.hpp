#ifndef DVRK_ARM_PARAMS_
#define DVRK_ARM_PARAMS_

#include <iostream>
#include <string>

class DVRKArmTypes {
  public:
    // Enum value DECLARATIONS - they are defined later
    static const DVRKArmTypes MTML;
    static const DVRKArmTypes MTMR;
    static const DVRKArmTypes PSM1;
    static const DVRKArmTypes PSM2;
    static const DVRKArmTypes ECM;

  private:

    const std::string name;
    const int dof;

  private:
    DVRKArmTypes( std::string name,   int dof): name(name), dof(dof) { }

  public:
    std::string getName() const {return std::string(name);}
    int getDof() const { return dof;}
    
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
};

#endif
