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
};

#endif
