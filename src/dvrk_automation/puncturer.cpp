/*
 *  puncturer.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-12-03
 *  
 */

#include "dvrk_automation/puncturer.hpp"
#include "dvrk/trajectory_factory.hpp"
#include <numeric>
#include <chrono>


namespace dvrk_automation {

// Constants
const double forceSurface = 50.0;	//mN
const double forceMaxPermitted = 5000.0;	//mN
const double travelHeight = 20.0;	//mm
const double workSpaceMarginBottom = 5.0; //mm
const double workSpaceMarginGeneral = 40.0;	//mm
const double raiseSpeedAir = 40.0; // mm/s
const double raiseSpeedTissue = 5.0; // mm/s
const double lowerSpeedAir = 10.0; // mm/s
const double travelSpeed  = 20.0; // mm/s

Puncturer::Puncturer(ros::NodeHandle nh, dvrk::ArmTypes arm_typ): 
							nh(nh), psm(nh, arm_typ, dvrk::PSM::ACTIVE)
							, oforce(nh)
{
	// Skip invalid points
	ros::Rate loop_rate(50.0);
	while (ros::ok() && psm.getPoseCurrent().position.norm() < 0.001)
  	{
  		loop_rate.sleep();
  	}
  	
  	// Set jaw pos
	constJawPos = psm.getPoseCurrent().jaw;
	
}

Puncturer::~Puncturer()
{
	// TODO Auto-generated destructor stub
}

void Puncturer::doPunctureSeries(Eigen::Vector2d scanningArea,
					Eigen::Vector2i nLocations, int nTrials,
					double depth, double speed, double T, double dt)
{
	try {
		//init
		ROS_INFO_STREAM("Initializing automatic tissue puncture...");
		// Wait while optoforce and dvrk send messages
		ros::Rate loop_rate(1.0/dt);
		while (ros::ok() && psm.getPoseCurrent().position.norm() < 0.001
			&& oforce.getForcesCurrent().norm() < 0.1)
  		{
  			loop_rate.sleep();
  		}
	
		// Go to init pos: raise to travel height, calculate safety params
		setWorkspaceFromCurrent(scanningArea);
		raiseToTravelHeight();
		ros::Duration(0.5).sleep();
	
		// Find surface, refine safety params
		ROS_INFO_STREAM("Finding surface of tissue...");
		findTissueSurface();
		ros::Duration(0.5).sleep();
		setWorkspaceFromCurrent(scanningArea);
		dvrk::Pose p0 = psm.getPoseCurrent();
		Eigen::Vector2d start_location(p0.position.x(), p0.position.y());
		ROS_INFO_STREAM("Surface found, starting automatic tissue puncture...");
				
		// For each location
		Eigen::Vector2d location_step;
		location_step(0) = scanningArea.x() / (nLocations.x() -1);
		location_step(1) = scanningArea.y() / (nLocations.y() -1);
				
		for (int i = 0; i < nLocations.x(); i++)
			for (int j = 0; j < nLocations.y(); j++)
			{
				Eigen::Vector2d curr_loc_step(i*location_step.x(),
												 j*location_step.x());
				Eigen::Vector2d curr_location = start_location + curr_loc_step;
				goToLocation(curr_location);
				ROS_INFO_STREAM("Surface found, starting automatic tissue puncture...");
				ros::Duration(0.5).sleep();
				// Go to pos
		
				// Find surface
		
				// For nTrials
		
				puncture();
			}
				
	} catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error ...");
  	}

}

void Puncturer::setWorkspaceFromCurrent(Eigen::Vector2d scanningArea)
{
	dvrk::Pose p0 = psm.getPoseCurrent();
	workspaceLowerBounds(2) = p0.position.z() - workSpaceMarginBottom;
	workspaceUpperBounds(2) = p0.position.z() + workSpaceMarginGeneral;
	if (scanningArea.x() >= 0.0)
	{
		workspaceLowerBounds(0) = p0.position.x() - workSpaceMarginGeneral;
		workspaceUpperBounds(0) = p0.position.x() + scanningArea.x()
									+ workSpaceMarginGeneral;
	} else {
		workspaceUpperBounds(0) = p0.position.x() - workSpaceMarginGeneral;
		workspaceLowerBounds(0) = p0.position.x() + scanningArea.x()
									+ workSpaceMarginGeneral;
	}
	if (scanningArea.y() >= 0.0)
	{
		workspaceLowerBounds(1) = p0.position.y() - workSpaceMarginGeneral;
		workspaceUpperBounds(1) = p0.position.y() + scanningArea.y()
									+ workSpaceMarginGeneral;
	} else {
		workspaceUpperBounds(1) = p0.position.y() - workSpaceMarginGeneral;
		workspaceLowerBounds(1) = p0.position.y() + scanningArea.y()
									+ workSpaceMarginGeneral;
	}
}

void Puncturer::raiseToTravelHeight()
{
	dvrk::Pose p0 = psm.getPoseCurrent();
	dvrk::Pose p1 = p0;
	p1.jaw = constJawPos;
	Eigen::Vector3d translate_to_travel_height(0.0, 0.0, travelHeight);
	p1.position += translate_to_travel_height;
	// TODO set orientation
	dvrk::Trajectory<dvrk::Pose> to_init(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					p1,
					(travelHeight / raiseSpeedAir), 0.1, dt));
	checkTrajectory(to_init);
	psm.playTrajectory(to_init);
}

void Puncturer::goToLocation(Eigen::Vector2d location)
{

	dvrk::Pose p0 = psm.getPoseCurrent();
	dvrk::Pose p1 = p0;
	p1.position(0) = location.x();
	p1.position(1) = location.y();
	p1.jaw = constJawPos;
	// TODO set orientation
	dvrk::Trajectory<dvrk::Pose> to_location(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					p1,
					(p1.dist(p0).cartesian / travelSpeed), 0.1, dt));
	checkTrajectory(to_location);
	psm.playTrajectory(to_location);
}

void Puncturer::findTissueSurface()
{
	Eigen::Vector3d step(0.0, 0.0, lowerSpeedAir * dt);
	dvrk::Pose p0 = psm.getPoseCurrent();
	dvrk::Pose p1 = p0;
	while (ros::ok() && oforce.getForcesCurrent().norm() < forceSurface)
	{
		p1 -= step;
		checkPose(p1);
		psm.moveCartesianAbsolute(p1);	
	}
}

void Puncturer::puncture()
{
	// Move down with given speed to given depth, do safety check
	
	// Log results to file, one file for each location
}

void Puncturer::safetyCheck()
{
	// Check position, jaw and force
}


void Puncturer::checkTrajectory(dvrk::Trajectory<dvrk::Pose> tr)
{
	for (int i = 0; i < tr.size(); i++)
		checkPose(tr[i]);
}

void Puncturer::checkPose(dvrk::Pose p)
{
	Eigen::Vector3d position = p.position;
	double jaw = p.jaw;
	if (position.x() < workspaceLowerBounds.x() ||
		position.x() > workspaceUpperBounds.x() ||
		position.y() < workspaceLowerBounds.y() ||
		position.y() > workspaceUpperBounds.y() ||
		position.z() < workspaceLowerBounds.z() ||
		position.z() > workspaceUpperBounds.z())
	{
		std::stringstream errstring;
    	errstring << "Desired position is out of workspace." 
    					<< std::endl
    					<< "Desired position:\t" 
    					<< position
    					<< std::endl
    					<< "Workspace lower bounds:\t" 
    					<< workspaceLowerBounds
    					<< std::endl
    					<< "Workspace upper bounds:\t" 
    					<< workspaceUpperBounds
    					<< std::endl;
    	//ROS_ERROR_STREAM(errstring.str());
		throw std::runtime_error(errstring.str());
	}
	if (jaw < (constJawPos - 0.01) ||
		jaw > (constJawPos + 0.01))
	{
		std::stringstream errstring;
    		errstring << 
    		"Tried to move jaw, which is permitted during puncture trials." 
    					<< std::endl
    					<< "Desired jaw angle:\t" 
    					<< jaw
    					<< std::endl
    					<< "Constant jaw angle:\t" 
    					<< constJawPos
    					<< std::endl;
    	//ROS_ERROR_STREAM(errstring.str());
		throw std::runtime_error(errstring.str());
	}
}



}



