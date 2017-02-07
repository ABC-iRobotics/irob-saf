/*
 * dvrk_arm.cpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#include "dvrk_arm_types.hpp"

// Enum value DEFINITIONS
// The initialization occurs in the scope of the class,
// so the private DVRKArmParams constructor can be used.
    
// TODO set maxvels for MTM-s and ECM
const DVRKArmTypes DVRKArmTypes::MTML = DVRKArmTypes("MTML", 8, 
	{0.01, 5.0, std::numeric_limits<double>::infinity()},
	{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
const DVRKArmTypes DVRKArmTypes::MTMR = DVRKArmTypes("MTMR", 8, 
	{0.01, 5.0, std::numeric_limits<double>::infinity()},
	{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01});
const DVRKArmTypes DVRKArmTypes::PSM1 = DVRKArmTypes("PSM1", 7, 
	{0.3, 250.0, 15.0},
	{8.0, 25.0, 0.6, 6.0, 6.0, 9.0, 15.0});
const DVRKArmTypes DVRKArmTypes::PSM2 = DVRKArmTypes("PSM2", 7, 
	{0.3, 250.0, 15.0},
	{8.0, 25.0, 0.6, 6.0, 6.0, 9.0, 60.0});
const DVRKArmTypes DVRKArmTypes::ECM = DVRKArmTypes("ECM", 6, 
	{0.01, 5.0, std::numeric_limits<double>::infinity()},
	{0.01, 0.01, 0.01, 0.01, 0.01, 0.01});


