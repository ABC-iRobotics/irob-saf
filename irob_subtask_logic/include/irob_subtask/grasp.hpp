/*
 * 	grasp.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *
 *  Grasp the object received in the topic 'saf/vision/target'.
 *
 */

#ifndef GRASP_HPP_
#define GRASP_HPP_

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

#include <irob_subtask/autosurg_agent.hpp>


namespace saf {

class Grasp : public AutosurgAgent {


protected:

  VisionClient<geometry_msgs::Pose, Pose> vision;


public:
  Grasp(ros::NodeHandle, std::vector<std::string>);
  ~Grasp();
  void graspObject();

};

}
#endif /* GRASP_HPP_ */
