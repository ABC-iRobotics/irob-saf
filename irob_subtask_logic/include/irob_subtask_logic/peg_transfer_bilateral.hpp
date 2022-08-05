/*
 * 	peg_transfer_dual.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-06-01
 *
 *  Prototype of the autonomous peg transfer
 *  training exercise with two arms.
 *
 */

#ifndef PEG_TRANSFER_BILATERAL_HPP
#define PEG_TRANSFER_BILATERAL_HPP


#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>

#include <irob_utils/tool_pose.hpp>
#include <irob_utils/utils.hpp>
#include <irob_msgs/GraspObject.h>
#include <irob_utils/abstract_directions.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>
#include <irob_subtask_logic/peg_transfer_logic.hpp>


namespace saf {

class PegTransferBilateral : public PegTransferLogic {


public:
  PegTransferBilateral(ros::NodeHandle, ros::NodeHandle, std::vector<std::string>);
  ~PegTransferBilateral();
  void doPegTransfer();


};

}

#endif // PEG_TRANSFER_BILATERAL_HPP
