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
    
const DVRKArmTypes DVRKArmTypes::MTML = DVRKArmTypes("MTML", 8);
const DVRKArmTypes DVRKArmTypes::MTMR = DVRKArmTypes("MTMR", 8);
const DVRKArmTypes DVRKArmTypes::PSM1 = DVRKArmTypes("PSM1", 7);
const DVRKArmTypes DVRKArmTypes::PSM2 = DVRKArmTypes("PSM2", 7);
const DVRKArmTypes DVRKArmTypes::ECM = DVRKArmTypes("ECM", 6);


