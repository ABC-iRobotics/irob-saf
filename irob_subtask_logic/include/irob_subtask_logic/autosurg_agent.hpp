/*
 * 	autosurg_agent.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *
 *  Base class for autonomous surgical agents.
 *  Possesses a vector that can contans SurgemeClient.
 *
 */

#ifndef AUTOSURG_AGENT_HPP_
#define AUTOSURG_AGENT_HPP_

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
#include <irob_motion/surgeme_client.hpp>
#include <irob_sensory_support/sensory_client.hpp>


namespace saf {

class AutosurgAgent {

protected:
  ros::NodeHandle nh;

  std::vector<SurgemeClient*> arms;

  double speed_cartesian;
  double speed_jaw;

  // Inherited classes should countain 1 or more
  // VisionClient, e. g.:
  // VisionClient<geometry_msgs::Point, Eigen::Vector3d> vision;



public:
  AutosurgAgent(ros::NodeHandle nh, ros::NodeHandle priv_nh, std::vector<std::string> arm_names): nh(nh)
  {

    // Get params from launch
    priv_nh.getParam("speed_cartesian", speed_cartesian);
    priv_nh.getParam("speed_jaw", speed_jaw);

    for (std::string name : arm_names)
      arms.push_back(new SurgemeClient(nh, name));
  }

  ~AutosurgAgent()
  {
    for (SurgemeClient* gc : arms)
      delete(gc);
  }


};

}
#endif /* AUTOSURG_AGENT_HPP_ */
