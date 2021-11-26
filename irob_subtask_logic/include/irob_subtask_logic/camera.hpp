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

#include <irob_msgs/VisionObject.h>
#include <irob_msgs/VisionObjectArray.h>

#include <irob_utils/tool_pose.hpp>
#include <irob_utils/utils.hpp>
#include <irob_utils/abstract_directions.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>


namespace saf {

class Camera : public AutosurgAgent {


protected:

  VisionClient<irob_msgs::VisionObjectArray,irob_msgs::VisionObjectArray> vision;
  double speed_carthesian;
  double marker_dist_threshold;
  double marker_dist_desired;
  double marker_threshold;
  double camera_offset_x;
  double camera_offset_y;

public:
  Camera(ros::NodeHandle, ros::NodeHandle, std::vector<std::string>, double, double, double, double, double, double);
  ~Camera();
  void moveCam();

};

}
#endif /* CAMERA_HPP_ */
