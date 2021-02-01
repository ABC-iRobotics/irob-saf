
/*
 * 	cut_vessel.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-06-04
 *
 *  Vessel cutting exercise
 *  with two arms.
 *
 */


#ifndef CUT_VESSEL_HPP
#define CUT_VESSEL_HPP


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


namespace saf {

class CutVessel : public AutosurgAgent {


protected:

  VisionClient<geometry_msgs::Transform, Eigen::Affine3d> vision;

  Eigen::Vector3d board_t;
  std::vector<Eigen::Vector3d> vessel_ends;
  double object_h;
  double object_d;


  void loadBoardDescriptor(ros::NodeHandle);
  Eigen::Affine3d poseToCameraFrame(const Eigen::Affine3d&, const Eigen::Affine3d&);
  Eigen::Affine3d poseToWorldFrame(const Eigen::Affine3d&, const Eigen::Affine3d&);


public:
  CutVessel(ros::NodeHandle, ros::NodeHandle, std::vector<std::string>);
  ~CutVessel();
  void doVesselCutting();

};

}



#endif // CUT_VESSEL_HPP
