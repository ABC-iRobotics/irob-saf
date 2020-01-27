/*
 * 	palpate.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2020-01-27
 *
 *  Simple soft-tissue palpation.
 *
 */

#ifndef PALPATE_HPP_
#define PALPATE_HPP_

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
#include <irob_sensory_support/sensory_client.hpp>
#include <irob_sensory_support/optoforce_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>


namespace saf {

class Palpate : public AutosurgAgent {


protected:

  OptoforceClient optoforce;
  double dt;


public:
  Palpate(ros::NodeHandle, ros::NodeHandle priv_nh, std::vector<std::string>);
  ~Palpate();
  void palpateSample(std::string);
  void writeData(std::string, std::string,
            std::vector<double>&, std::vector<double>&);

};

}
#endif /* PALPATE_HPP_ */
