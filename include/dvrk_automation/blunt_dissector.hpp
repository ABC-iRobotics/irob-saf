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


namespace dvrk_automation {

class BluntDissector {
   

private:
    ros::NodeHandle nh;

    // States
    dvrk::PSM psm;
    dvrk_vision::VisionConn vision;
    
    
    double dt;
	
	// Constants
	//static const double travelSpeed;
    
public:
	BluntDissector(ros::NodeHandle, dvrk::ArmTypes, double);
	~BluntDissector();
	
	void dissect();
	
	void goToTarget(double stepT, double speed = 10.0);
	void toolPushIn(double depth, double speed  = 2.0);
	void toolPullOut(double depth, double speed  = 2.0);
	void toolOpen(double angle, double speed  = 10.0);
	void toolClose(double angle = 0.0, double speed  = 10.0);
	void toolRotate(double angle, double speed = 10.0);
	
	void safetyCheck();
	void checkTrajectory(dvrk::Trajectory<dvrk::Pose>);
	void checkPose(dvrk::Pose);
	void checkPoseCurrent();

private:
	void waitForTopicsInit();
	
};

}


#endif /* DVRK_BLUNT_DISSECTOR_HPP_ */
