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
#include <geometry_msgs/Transform.h>

#include <irob_utils/pose.hpp>
#include <irob_utils/utils.hpp>
#include <irob_msgs/GraspObject.h>
#include <irob_utils/abstract_directions.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_sensory_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>


namespace saf {

class PegTransferDual : public AutosurgAgent {


protected:

  VisionClient<geometry_msgs::Transform, geometry_msgs::Transform> vision;

  Eigen::Vector3d board_t;
  std::vector<Eigen::Vector3d> peg_positions;
  double peg_h;
  double object_h;
  double object_d;
  double object_wall_d;


  void loadBoardDescriptor(ros::NodeHandle);
  Pose poseToCameraFrame(const Pose&, const geometry_msgs::Transform&);
  Pose poseToWorldFrame(const Pose&, const geometry_msgs::Transform&);


public:
  PegTransferDual(ros::NodeHandle, ros::NodeHandle, std::vector<std::string>);
  ~PegTransferDual();
  void doPegTransfer();

};

}

#endif // PEG_TRASNFER_DUAL_HPP
