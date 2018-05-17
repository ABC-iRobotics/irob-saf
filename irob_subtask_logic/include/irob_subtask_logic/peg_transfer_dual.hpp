/*
 * 	peg_transfer_dual.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018_05_07
 *
 *  Prototype of the autonomous peg transfer
 *  training exercise.
 *
 */

#ifndef PEG_TRASNFER_DUAL_HPP
#define PEG_TRASNFER_DUAL_HPP


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

#include <irob_utils/pose.hpp>
#include <irob_utils/utils.hpp>
#include <irob_msgs/Environment.h>
#include <irob_msgs/GraspObject.h>
#include <irob_utils/abstract_directions.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>


namespace saf {

class PegTransferDual : public AutosurgAgent {


protected:

  VisionClient<irob_msgs::Environment, irob_msgs::Environment> vision;



public:
  PegTransferDual(ros::NodeHandle, std::vector<std::string>);
  ~PegTransferDual();
  void doPegTransfer();

};

}

#endif // PEG_TRASNFER_DUAL_HPP
