/*
 *  blunt_dissector.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-02-08
 *  
 */

#include "dvrk_automation/blunt_dissector.hpp"
#include "dvrk/trajectory_factory.hpp"
#include <numeric>
#include <chrono>


namespace dvrk_automation {

// Constants
//const double BluntDissector::travelSpeed  = 20.0 / 1000.0; // m/s

BluntDissector::BluntDissector(ros::NodeHandle nh, dvrk::ArmTypes arm_typ, double dt): 
							nh(nh), psm(nh, arm_typ, dvrk::PSM::ACTIVE), dt(dt)
{
		psm.setRobotState(dvrk::PSM::STATE_POSITION_CARTESIAN);
		waitForTopicsInit();
}

BluntDissector::~BluntDissector()
{
	// TODO Auto-generated destructor stub
}

void BluntDissector::dissect()
{
	try {	// TODO err handling
		
				
	} catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error ...");
  	}

}

void BluntDissector::waitForTopicsInit()
{
	ros::Rate loop_rate(1.0/dt);
	while (ros::ok() && psm.getPoseCurrent().position.norm() < 0.001
			/*&& oforce.getForcesCurrent().norm() < 0.1*/)
  	{
  		loop_rate.sleep();
  	}
}


void BluntDissector::goToTarget()
{
	dvrk::Trajectory<dvrk::Pose> to_target;
	//TODO 

	checkTrajectory(to_target);
	psm.playTrajectory(to_target);
}

/**
 * Push tool between tissues in the current direction.
 *
 * @param depth insertion depth in mm
 * @param speed insertion speed in mm/s
 */
void  BluntDissector::toolPushIn(double depth, double speed /* = 2.0 */)
{
	Eigen::Vector3d v(0,0,1);
	dvrk::Pose p1 = psm.getPoseCurrent();
	Eigen::Matrix3d R = p1.orientation.toRotationMatrix();
	v = R*v;
	dvrk::Pose p2 = p1+((std::abs(depth)/1000.0)*v);
	double T = std::abs(depth / speed);
	dvrk::Trajectory<dvrk::Pose> tr = dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);
   					
   	// Safety
   	checkTrajectory(tr);						
   						
   	psm.playTrajectory(tr); 
}

/**
 * Push tool out of tissues in the current direction.
 *
 * @param depth pull distance in mm
 * @param speed pull speed in mm/s
 */	
void  BluntDissector::toolPullOut(double depth, double speed /* = 2.0 */)
{
	Eigen::Vector3d v(0,0,1);
	dvrk::Pose p1 = psm.getPoseCurrent();
	Eigen::Matrix3d R = p1.orientation.toRotationMatrix();
	v = R*v;
	dvrk::Pose p2 = p1-((std::abs(depth)/1000.0)*v);
	double T = std::abs(depth / speed);
	dvrk::Trajectory<dvrk::Pose> tr = dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);
   	
   	// Safety
   	checkTrajectory(tr);						
   						
   	psm.playTrajectory(tr); 
}

/**
 * Open tool to separate tissues.
 *
 * @param angle angle of jaws in deg
 * @param speed opening speed in deg/s
 */	
void  BluntDissector::toolOpen(double angle, double speed /* = 10.0 */)
{
	dvrk::Pose p1 = psm.getPoseCurrent();
	
	double angle_rad = (std::abs(angle)/360.0)* M_PI * 2.0;
	double speed_rad = (std::abs(speed)/360.0)* M_PI * 2.0;
		
	double T = std::abs((angle_rad-p1.jaw) / speed_rad);
	
		
	dvrk::Pose p2 = p1;
	p2.jaw = angle_rad;
	
	dvrk::Trajectory<dvrk::Pose> tr = dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);
   	
   	// Safety
   	checkTrajectory(tr);						
   						
   	psm.playTrajectory(tr); 
}

/**
 * Close tool after separated tissues.
 *
 * @param angle angle of jaws in deg, default is fully closed
 * @param speed closing speed in deg/s
 */	
void  BluntDissector::toolClose(double angle /* = 0.0 */, 
								double speed /* = 10.0 */)
{
	dvrk::Pose p1 = psm.getPoseCurrent();
	
	double angle_rad = (std::abs(angle)/360.0)* M_PI * 2.0;
	double speed_rad = (std::abs(speed)/360.0)* M_PI * 2.0;
		
	double T = std::abs((angle_rad-p1.jaw) / speed_rad);
		
	dvrk::Pose p2 = p1;
	p2.jaw = angle_rad;
	
	dvrk::Trajectory<dvrk::Pose> tr = dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);
   	// Safety
   	checkTrajectory(tr);					
   						
   	psm.playTrajectory(tr); 
}

/**
 * Rotate tool.
 *
 * @param angle angle of rotation
 * @param speed rotation speed in deg/s
 */	
void  BluntDissector::toolRotate(double angle, double speed /* = 10.0 */)
{
	double angle_rad = (angle/360.0)* M_PI * 2.0;
	double speed_rad = (speed/360.0)* M_PI * 2.0;
		
	double T = std::abs(angle_rad / speed_rad);

	dvrk::Pose p1 = psm.getPoseCurrent();
	Eigen::Vector3d v(0,0,1);
	Eigen::Matrix3d R = p1.orientation.toRotationMatrix();
	v = R*v;
	v.normalize();
	Eigen::AngleAxis<double> t(Eigen::AngleAxis<double>(angle_rad,v));
	
	Eigen::Quaternion<double> ori = p1.orientation;
	
	ori = t * ori;
	
	dvrk::Pose p2 = p1;
	p2.orientation = ori;
	
	dvrk::Trajectory<dvrk::Pose> tr = dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);
   	// Safety
   	checkTrajectory(tr);					
   						
   	psm.playTrajectory(tr); 
}


void BluntDissector::safetyCheck()
{
	// Check position, jaw and force
	checkPoseCurrent();
	
}


void  BluntDissector::checkTrajectory(dvrk::Trajectory<dvrk::Pose> tr)
{
	for (int i = 0; i < tr.size(); i++)
		checkPose(tr[i]);
}

void  BluntDissector::checkPoseCurrent()
{
	checkPose(psm.getPoseCurrent());
}



void  BluntDissector::checkPose(dvrk::Pose p)
{
	Eigen::Vector3d position = p.position;
	
	//TODO
	
	//throw std::runtime_error(errstring.str());
}




}
