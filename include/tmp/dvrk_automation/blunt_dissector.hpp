/*
 * 	blunt_dissector.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-02-08
 *
 */

#ifndef DVRK_BLUNT_DISSECTOR_HPP_
#define DVRK_BLUNT_DISSECTOR_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cmath> 
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include "dvrk/arm_types.hpp"
#include "dvrk/topics.hpp"
#include "dvrk/pose.hpp"
#include "dvrk/trajectory.hpp"
#include "dvrk/utils.hpp"
#include "dvrk/arm.hpp"
#include "dvrk/psm.hpp"
#include "dvrk_automation/subtask_status.hpp"

#include "dvrk_vision/vision_conn.hpp"
#include "dvrk_vision/target_type.hpp"
#include "dvrk_vision/position_type.hpp"


namespace dvrk_automation {

class BluntDissector {
   

private:
    ros::NodeHandle nh;

    // States
    dvrk::PSM dissector_arm;
    dvrk_vision::VisionConn dissector_vision;
    
    // States
    dvrk::PSM retractor_arm;
    dvrk_vision::VisionConn retractor_vision;
    
    
    double dt;
	
	// Constants
	//static const double travelSpeed;
    
public:
	BluntDissector(ros::NodeHandle, dvrk::ArmTypes, std::string,
				 dvrk::ArmTypes, std::string, double);
	~BluntDissector();
	
	void dissect();
	
	void goToTarget(dvrk::PSM &arm,  dvrk_vision::VisionConn &vision,
		dvrk_vision::TargetType target_type, double speed = 10.0);
	void toolPushIn(dvrk::PSM &arm,  dvrk_vision::VisionConn &vision,
		double depth, double speed  = 2.0);
	void toolPullOut(dvrk::PSM &arm,  dvrk_vision::VisionConn &vision,
		double depth, double speed  = 2.0);
	void toolOpen(dvrk::PSM &arm,  dvrk_vision::VisionConn &vision,
		double angle, double speed  = 10.0);
	void toolClose(dvrk::PSM &arm,  dvrk_vision::VisionConn &vision,
		double angle = 0.0, double speed  = 10.0);
	void toolRotate(dvrk::PSM &arm,  dvrk_vision::VisionConn &vision,
		double angle, double speed = 10.0);
	
	void safetyCheck();
	void checkTrajectory(dvrk::Trajectory<dvrk::Pose>);
	void checkPose(dvrk::Pose);
	void checkPoseCurrent();

private:
	void waitForTopicsInit();
	
};

}


#endif /* DVRK_BLUNT_DISSECTOR_HPP_ */