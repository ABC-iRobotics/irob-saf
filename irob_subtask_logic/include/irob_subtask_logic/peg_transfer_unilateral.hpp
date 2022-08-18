/*
 * 	peg_transfer.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *
 *  Prototype of the autonomous peg transfer
 *  training exercise.
 *
 */

#ifndef PEG_TRANSFER_UNILATERAL_HPP
#define PEG_TRANSFER_UNILATERAL_HPP


#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <limits>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>

#include <irob_utils/tool_pose.hpp>
#include <irob_utils/utils.hpp>
#include <irob_msgs/GraspObject.h>
#include <irob_msgs/Environment.h>
#include <irob_utils/abstract_directions.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>
#include <irob_subtask_logic/peg_transfer_logic.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace saf {

class PegTransferUnilateral : public PegTransferLogic {

  double offs_x;
  double offs_y;
  double offs_z;

  std::string offset_filename_arm_1;
  std::string measurement_filename;

public:
  PegTransferUnilateral(ros::NodeHandle, ros::NodeHandle, std::vector<std::string>);
  ~PegTransferUnilateral();
  void doPegTransfer();
  void calibrateOffset();
  void measureAccuracyBlocks();
  void measureAccuracyPegs();

};

}

#endif // PEG_TRANSFER_UNILATERAL_HPP
