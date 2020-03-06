/*
 * 	grasp.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *
 *  Grasp the object received in the topic 'saf/vision/target'.
 *
 */

#ifndef CAMERA_HPP_
#define CAMERA_HPP_

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
#include <irob_utils/abstract_directions.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>


namespace saf {

class Camera : public AutosurgAgent {


protected:

  VisionClient<geometry_msgs::Point,Eigen::Vector3d> vision;
  double speed_carthesian;
  double marker_dist_threshold;
  double marker_dist_desired;
  double marker_xy_threshold;

public:
  Camera(ros::NodeHandle, ros::NodeHandle, std::vector<std::string>, double, double, double, double);
  ~Camera();
  void moveCam();

};

}
#endif /* CAMERA_HPP_ */
