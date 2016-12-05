/*
 * 	puncturer.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-03
 *
 */

#ifndef DVRK_PUNCTURER_HPP_
#define DVRK_PUNCTURER_HPP_

#include <iostream>
#include <sstream>
#include <vector>
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
#include "optoforce/optoforce_listener.hpp"


namespace dvrk_automation {

class Puncturer {
   

private:
    ros::NodeHandle nh;

    // States
    dvrk::PSM psm;
    optoforce::OptoforceListener oforce;
    
    double dt;
    
    // Safety params
    Eigen::Vector3d workspaceLowerBounds;
    Eigen::Vector3d workspaceUpperBounds;

	// Never ever change jaw pos
	double constJawPos;
	
	// Constants
	static const double forceSurface;
	static const double forceMaxPermitted;
	static const double travelHeight;
	static const double surfaceOffset;
	static const double workSpaceMarginBottom;
	static const double workSpaceMarginGeneral;
	static const double raiseSpeedAir;
	static const double raiseSpeedTissue;
	static const double lowerSpeedAir;
	static const double travelSpeed;
    
public:
	Puncturer(ros::NodeHandle, dvrk::ArmTypes, double);
	~Puncturer();
	
	void doPunctureSeries(Eigen::Vector2d, Eigen::Vector2i, int,
					double, double, double, std::string);
	
	void puncture(double, double, double,
						std::string, std::string);
	
	void raiseToSurface(double );
	void safetyCheck();
	void checkTrajectory(dvrk::Trajectory<dvrk::Pose>);
	void checkPose(dvrk::Pose);
	void checkPoseCurrent();
	void checkForcesCurrent();

private:
	void setWorkspaceFromCurrent(Eigen::Vector2d, double);
	void raiseToTravelHeight();
	void goToLocation(Eigen::Vector2d);
	void findTissueSurface();
	void waitForTopicsInit();
	
	/*
	void checkVelCartesian(const Pose&, const Pose&, double);
	void checkVelJoint(const sensor_msgs::JointState&, 
						const std::vector<double>&, double);
	*/
	
};

}


#endif /* DVRK_PUNCTURER_HPP_ */
