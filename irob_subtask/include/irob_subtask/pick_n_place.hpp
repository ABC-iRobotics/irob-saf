/*
 * 	pick_n_place.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *
 */

#ifndef PICK_N_PLACE_HPP_
#define PICK_N_PLACE_HPP_

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
#include <irob_motion/gesture_client.hpp>
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask/autosurg_agent.hpp>


namespace ias {

class PicknPlace : public AutosurgAgent {


protected:

    VisionClient<geometry_msgs::Pose, Pose> vision;
    

public:
	PicknPlace(ros::NodeHandle, std::vector<std::string>);
	~PicknPlace();
	 void graspObject();
	
};

}
#endif /* PICK_N_PLACE_HPP_ */
