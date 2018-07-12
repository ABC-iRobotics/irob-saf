
/*
 * 	accuracy_meas.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-07-12
 *
 *  Prototype of the autonomous peg transfer
 *  training exercise.
 *
 */

#ifndef ACCURACY_MEAS_HPP
#define ACCURACY_MEAS_HPP


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
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>


namespace saf {

class AccuracyMeas : public AutosurgAgent {


protected:

  VisionClient<geometry_msgs::Transform, geometry_msgs::Transform> vision;

  std::vector<Eigen::Vector3d> peg_positions;



public:
  AccuracyMeas(ros::NodeHandle, ros::NodeHandle, std::vector<std::string>);
  ~AccuracyMeas();
  void doAccuracyMeas();

};

}


#endif // ACCURACY_MEAS_HPP
