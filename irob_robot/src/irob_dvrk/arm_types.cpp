/*
 *  arm_types.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *
 */

#include <irob_dvrk/arm_types.hpp>

namespace saf {

// Enum value DEFINITIONS
// The initialization occurs in the scope of the class,
// so the private DVRKArmParams constructor can be used.
    
// TODO set maxvels for MTM-s and ECM
const ArmTypes ArmTypes::MTML = ArmTypes("MTML", 8, 
	{1.0, 1000.0, std::numeric_limits<double>::infinity()},
	{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
const ArmTypes ArmTypes::MTMR = ArmTypes("MTMR", 8, 
	{1.0, 1000.0, std::numeric_limits<double>::infinity()},
	{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
const ArmTypes ArmTypes::PSM1 = ArmTypes("PSM1", 7, 
  {10.0, 40000.0, 300.0},
  {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001});
const ArmTypes ArmTypes::PSM2 = ArmTypes("PSM2", 7, 
  {10.0, 40000.0, 300.0},
	{8.0, 25.0, 0.6, 6.0, 6.0, 9.0, 15.0});
const ArmTypes ArmTypes::PSM3 = ArmTypes("PSM3", 7,
  {10.0, 40000.0, 300.0},
  {8.0, 25.0, 0.6, 6.0, 6.0, 9.0, 15.0});
const ArmTypes ArmTypes::ECM = ArmTypes("ECM", 6, 
	{1.0, 1000.0, std::numeric_limits<double>::infinity()},
	{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});


}
