/*
 *  puncturer.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-03
 *  
 */

#include "dvrk_automation/puncturer.hpp"
#include <numeric>
#include <chrono>

namespace dvrk_automation {

Puncturer::Puncturer(ros::NodeHandle nh, dvrk::ArmTypes arm_typ): 
							nh(nh), psm(nh, arm_typ, dvrk::PSM::ACTIVE)
							, oforce(nh) {}

Puncturer::~Puncturer()
{
	// TODO Auto-generated destructor stub
}


}



