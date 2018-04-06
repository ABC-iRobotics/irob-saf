#ifndef PICK_N_PLACE_HPP
#define PICK_N_PLACE_HPP


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

#include <irob_subtask/autosurg_agent.hpp>


namespace ias {

class PicknPlace : public AutosurgAgent {


protected:

  VisionClient<irob_msgs::Environment, irob_msgs::Environment> vision;

  Eigen::Quaternion<double> ori;
  Eigen::Vector3d dp;


public:
  PicknPlace(ros::NodeHandle, std::vector<std::string>, Eigen::Quaternion<double>, Eigen::Vector3d);
  ~PicknPlace();
  void doPnP();

};

}

#endif // PICK_N_PLACE_HPP
