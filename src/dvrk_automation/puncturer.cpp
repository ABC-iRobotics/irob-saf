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
const double Puncturer::forceSurface = 50.0;	//mN
const double Puncturer::forceMaxPermitted = 5000.0;	//mN
const double Puncturer::travelHeight = 20.0 / 1000.0;	//m
const double Puncturer::surfaceOffset = 1.0 / 1000.0;	//m;
const double Puncturer::workSpaceMarginBottom = 10.0 / 1000.0; //m
const double Puncturer::workSpaceMarginGeneral = 40.0 / 1000.0;	//m
const double Puncturer::raiseSpeedAir = 40.0 / 1000.0; // m/s
const double Puncturer::raiseSpeedTissue = 5.0 / 1000.0; // m/s
const double Puncturer::lowerSpeedAir = 5.0 / 1000.0; // m/s // 
const double Puncturer::travelSpeed  = 20.0 / 1000.0; // m/s

Puncturer::Puncturer(ros::NodeHandle nh, dvrk::ArmTypes arm_typ, double dt): 
							nh(nh), psm(nh, arm_typ, dvrk::PSM::ACTIVE)
							, oforce(nh), dt(dt)
{
		psm.setRobotState(dvrk::PSM::STATE_POSITION_CARTESIAN);
}

Puncturer::~Puncturer()
{
	// TODO Auto-generated destructor stub
}

void Puncturer::doPunctureSeries(Eigen::Vector2d scanningArea,
					Eigen::Vector2i nLocations, int nTrials,
					double depth, double speed, double T, 
					std::string fileNameBase)
{
	try {	// TODO err handling
		//init
		ROS_INFO_STREAM("Initializing automatic tissue puncture...");
		// Skip invalid points
		ros::Rate loop_rate_init(50.0);
		while (ros::ok() && psm.getPoseCurrent().position.norm() < 0.001)
  		{
  			loop_rate_init.sleep();
  		}
  		// Set jaw pos
		constJawPos = psm.getPoseCurrent().jaw;
		// Wait while optoforce and dvrk send messages
		waitForTopicsInit();
	
		// Go to init pos: raise to travel height, calculate safety params
		setWorkspaceFromCurrent(scanningArea, depth);
		raiseToTravelHeight();
		ros::Duration(0.5).sleep();
		oforce.calibrateOffsets();
	
		// Find surface, refine safety params
		ROS_INFO_STREAM("Finding surface of tissue...");
		findTissueSurface();
		ros::Duration(0.5).sleep();
		setWorkspaceFromCurrent(scanningArea, depth);
		dvrk::Pose p0 = psm.getPoseCurrent();
		Eigen::Vector2d start_location(p0.position.x(), p0.position.y());
		ROS_INFO_STREAM("Surface found, starting automatic tissue puncture...");
				
		// For each location
		Eigen::Vector2d location_step;
		location_step(0) = scanningArea.x() / (nLocations.x() -1);
		location_step(1) = scanningArea.y() / (nLocations.y() -1);
				
		for (int j = 0; j < nLocations.y(); j++)
			for (int i = 0; i < nLocations.x(); i++)
			{
				// Go to pos
				raiseToTravelHeight();
				ros::Duration(0.5).sleep();
				
				Eigen::Vector2d curr_loc_step(i*location_step.x(),
												 j*location_step.y());
				Eigen::Vector2d curr_location = start_location + curr_loc_step;
				goToLocation(curr_location);
				ROS_INFO_STREAM("Location " << i << ", " << j);
				ros::Duration(0.5).sleep();
				// Calibrate optoforce
				oforce.calibrateOffsets();
				
				// Find surface
				ROS_INFO_STREAM("Finding surface of tissue...");
				findTissueSurface();
				double surface_z = psm.getPoseCurrent().position.z();
				ROS_INFO_STREAM("Surface found, starting " <<
								"series of trials...");
				ros::Duration(0.5).sleep();
				
				// For nTrials
				std::stringstream filename;
    			filename << fileNameBase << "_" << i <<	// TODO .dat
    										"_" << j; 
				for (int k = 0; k < nTrials; k++)
				{
					ROS_INFO_STREAM("Location " << i << ", " << j
										<< "\tTrial " << k << "/" << nTrials);
					std::stringstream comment;
					comment << "#" << k;
					puncture( depth, speed, T, filename.str(), comment.str());
					ros::Duration(0.5).sleep();
					raiseToSurface(surface_z);
					ros::Duration(0.5).sleep();
				}
			}
				
	} catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error ...");
  	}

}

void Puncturer::waitForTopicsInit()
{
	ros::Rate loop_rate(1.0/dt);
	while (ros::ok() && psm.getPoseCurrent().position.norm() < 0.001
			&& oforce.getForcesCurrent().norm() < 0.1)
  	{
  		loop_rate.sleep();
  	}
}

void Puncturer::setWorkspaceFromCurrent(Eigen::Vector2d scanningArea, 
										double depth)
{
	dvrk::Pose p0 = psm.getPoseCurrent();
	workspaceLowerBounds(2) = p0.position.z() - depth - workSpaceMarginBottom;
	workspaceUpperBounds(2) = p0.position.z() + travelHeight 
									+ workSpaceMarginGeneral;
	if (scanningArea.x() >= 0.0)
	{
		workspaceLowerBounds(0) = p0.position.x() - workSpaceMarginGeneral;
		workspaceUpperBounds(0) = p0.position.x() + scanningArea.x()
									+ workSpaceMarginGeneral;
	} else {
		workspaceUpperBounds(0) = p0.position.x() + workSpaceMarginGeneral;
		workspaceLowerBounds(0) = p0.position.x() + scanningArea.x()
									- workSpaceMarginGeneral;
	}
	if (scanningArea.y() >= 0.0)
	{
		workspaceLowerBounds(1) = p0.position.y() - workSpaceMarginGeneral;
		workspaceUpperBounds(1) = p0.position.y() + scanningArea.y()
									+ workSpaceMarginGeneral;
	} else {
		workspaceUpperBounds(1) = p0.position.y() + workSpaceMarginGeneral;
		workspaceLowerBounds(1) = p0.position.y() + scanningArea.y()
									- workSpaceMarginGeneral;
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

void Puncturer::raiseToSurface(double surface_z)
{
	dvrk::Pose p0 = psm.getPoseCurrent();
	dvrk::Pose p1 = p0;
	p1.jaw = constJawPos;
	p1.position(2) = surface_z;
	// TODO set orientation
	dvrk::Trajectory<dvrk::Pose> to_surface(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					p1,
					(p0.dist(p1).cartesian / raiseSpeedTissue), 0.1, dt));
	checkTrajectory(to_surface);
	psm.playTrajectory(to_surface);
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
	ros::Rate loop_rate(1.0/dt);
	while (ros::ok() && oforce.getForcesCurrent().norm() < forceSurface)
	{
		p1 -= step;
		checkPose(p1);
		psm.moveCartesianAbsolute(p1);
		loop_rate.sleep();
		safetyCheck();	
	}
	// Raise to offset
	p0 = psm.getPoseCurrent();
	p1 = p0;
	Eigen::Vector3d translate_to_surface(0.0, 0.0, surfaceOffset);
	p1.jaw = constJawPos;
	p1.position += translate_to_surface;
	// TODO set orientation
	dvrk::Trajectory<dvrk::Pose> to_surface(dvrk::TrajectoryFactory::
    			linearTrajectoryWithSmoothAcceleration(
    				psm.getPoseCurrent(), 
					p1,
					(p0.dist(p1).cartesian / raiseSpeedTissue), 0.01, dt));
	checkTrajectory(to_surface);
	psm.playTrajectory(to_surface);
}

void Puncturer::puncture(double depth, double speed, double T,
						std::string filename, std::string comment)
{
	// Move down with given speed to given depth, do safety check
	Eigen::Vector3d step(0.0, 0.0, speed * dt);
	dvrk::Pose p0 = psm.getPoseCurrent();
	double desired_z = p0.position.z() - depth;
	dvrk::Pose p1 = p0;
	ros::Rate loop_rate(1.0/dt);
	double t = 0.0;
	while (ros::ok() && p1.position.z() > desired_z)
	{
		// TODO collect data
		p1 -= step;
		checkPose(p1);
		psm.moveCartesianAbsolute(p1);
		safetyCheck();
		loop_rate.sleep();	
		t += dt;
	}
	while (ros::ok() && t <= T)
	{	
		// TODO collect data
		loop_rate.sleep();
		t += dt;
	}

	// TODO Log results to file, one file for each location
}

void Puncturer::safetyCheck()
{
	// Check position, jaw and force
	checkPoseCurrent();
	checkForcesCurrent();
	
}


void Puncturer::checkTrajectory(dvrk::Trajectory<dvrk::Pose> tr)
{
	for (int i = 0; i < tr.size(); i++)
		checkPose(tr[i]);
}

void Puncturer::checkPoseCurrent()
{
	checkPose(psm.getPoseCurrent());
}

void Puncturer::checkForcesCurrent()
{
	Eigen::Vector3d f = oforce.getForcesCurrent();
	if (f.x() > forceMaxPermitted ||
		f.y() > forceMaxPermitted ||
		f.z() > forceMaxPermitted ||
		f.norm() > forceMaxPermitted)
	{
		std::stringstream errstring;
    	errstring << "Measured force over limit." 
    					<< std::endl
    					<< "Current force:\t" 
    					<< f.transpose()
    					<< std::endl
    					<< "Max force permitted:\t" 
    					<< forceMaxPermitted
    					<< std::endl;
    	ROS_ERROR_STREAM("Force limit breached, lifting arm...");
    	raiseToTravelHeight();
		throw std::runtime_error(errstring.str());
	}
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
    					<< position.transpose()
    					<< std::endl
    					<< "Workspace lower bounds:\t" 
    					<< workspaceLowerBounds.transpose()
    					<< std::endl
    					<< "Workspace upper bounds:\t" 
    					<< workspaceUpperBounds.transpose()
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



