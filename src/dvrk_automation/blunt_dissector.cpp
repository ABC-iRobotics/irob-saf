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

BluntDissector::BluntDissector(	ros::NodeHandle nh,
				dvrk::ArmTypes dissector_arm_typ,
				std::string dissector_regfile_name,
				dvrk::ArmTypes retractor_arm_typ
				,std::string retractor_regfile_name,
				double dt): 
					nh(nh),
					dissector_arm(nh, dissector_arm_typ, dvrk::PSM::ACTIVE),
					dissector_vision(nh, dissector_regfile_name, "dissector"),
					retractor_arm(nh, retractor_arm_typ, dvrk::PSM::ACTIVE),
					retractor_vision(nh, retractor_regfile_name, "retractor"),
					dt(dt)
{
		dissector_arm.setRobotState(
			dvrk::PSM::STATE_POSITION_CARTESIAN);
		retractor_arm.setRobotState(
			dvrk::PSM::STATE_POSITION_CARTESIAN);
		waitForTopicsInit();
}

BluntDissector::~BluntDissector()
{
	// TODO Auto-generated destructor stub
}

void BluntDissector::dissect()
{
	try {	// TODO err handling
			
			// Grab tissue for retraction
			ROS_INFO_STREAM("Start tissue grabbing.");
			toolOpen(retractor_arm,
						 retractor_vision, 40.0, 20.0);
			
			goToTarget(retractor_arm,
						 retractor_vision,
						 dvrk_vision::TargetType::GRABBING,	// with DP
						 20.0);
			ros::Duration(0.01).sleep();
			
			
			toolClose(retractor_arm,
						 retractor_vision,5.0, 20.0);
			ros::Duration(0.01).sleep();
			ROS_INFO_STREAM("Tissue grabbed.");
			
			
			// Do initial retaction
			ROS_INFO_STREAM("Lifting the tissue.");
			goToTarget(retractor_arm,
						 retractor_vision,
						 dvrk_vision::TargetType::RETRACTION,
						 20.0);
			ros::Duration(0.01).sleep();
			
		
			// Do this while the vision says it is done
			ROS_INFO_STREAM("Starting dissection.");
			bool doneSomething = true;
			while (doneSomething)	
			{
				doneSomething = false;
				
				if (retractor_vision.needToDoTask()) {
				doneSomething = true;
				// Fine retraction
				ROS_INFO_STREAM("Starting fine retraction.");
				goToTarget(retractor_arm,
						 retractor_vision,
						 dvrk_vision::TargetType::RETRACTION,
						 20.0);
				ros::Duration(0.01).sleep();
				ROS_INFO_STREAM("Retraction done.");
				
				// Ask for new target
				ROS_INFO_STREAM("Ask for new target");
				dissector_vision.sendSubtaskStatus(
						SubtaskStatus::
						NEW_DISSECTION_TARGET_NEEDED.getCommand());
				}
				
				if (dissector_vision.needToDoTask()) {
				doneSomething = true;
				// Go to target
				ROS_INFO_STREAM("Go to target and make dissection.");
				goToTarget(dissector_arm,
						 dissector_vision,
						 dvrk_vision::TargetType::DISSECTION,
						 20.0);
				ros::Duration(0.01).sleep();
				dissector_vision.sendSubtaskStatus(SubtaskStatus::
						PERFORMING_DISSECTION.getCommand());
				
				// Make dissection	
				ROS_INFO_STREAM("Tool push in.");
				toolPushIn(dissector_arm,
						 dissector_vision,3.0, 2.0);
				ros::Duration(0.01).sleep();
				ROS_INFO_STREAM("Tool open.");
				toolOpen(dissector_arm,
						 dissector_vision,40.0, 20.0);
				ros::Duration(0.01).sleep();
				toolPullOut(dissector_arm,
						 dissector_vision,6.0, 5.0);
				ros::Duration(0.01).sleep();
				toolClose(dissector_arm,
						 dissector_vision,0.0, 20.0);
				ros::Duration(0.01).sleep();
				
				// Ask for new distant target
				ROS_INFO_STREAM("Ask for new distant target.");
				dissector_vision.sendSubtaskStatus(
						SubtaskStatus::
						NEW_DISTANT_TARGET_NEEDED.getCommand());
				
				// Go to distant target
				ROS_INFO_STREAM("Go to distant target.");
				goToTarget(dissector_arm,
						 dissector_vision,
						 dvrk_vision::TargetType::DISTANT,
						 20.0);
				ros::Duration(0.1).sleep();
				}
			}		
		
	} catch (const std::exception& e) {
		// Abort task
		dissector_vision.sendSubtaskStatus(SubtaskStatus::
						ABORT.getCommand());
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM(
  		 "Program stopped by an error, send ABORT command to vision node ...");
  	}

}

void BluntDissector::waitForTopicsInit()
{
	// TODO wait for vision init?
	ros::Rate loop_rate(1.0/dt);
	while (ros::ok() && dissector_arm.getPoseCurrent().position.norm() < 0.001
			/*&& oforce.getForcesCurrent().norm() < 0.1*/)
  	{
  		loop_rate.sleep();
  	}
}

/**
 * Push tool between tissues in the current direction.
 *
 * @param stepT time for planning subtrajectories in s
 * @param speed base movement speed in mm/s
 */
void BluntDissector::goToTarget(dvrk::PSM &arm,  dvrk_vision::VisionConn &vision,
								dvrk_vision::TargetType target_type, 
								double speed /* = 10.0 */)
{
	dvrk_vision::PositionType position_type = dvrk_vision::PositionType::DP;
		
	while (position_type == dvrk_vision::PositionType::DP)
	{
		// Get positions
		dvrk::Pose pose_current = arm.getPoseCurrent();
		dvrk::Pose end_target;

		vision.getTargetCurrent(target_type, end_target, position_type);

		// Go to position
		vision.sendSubtaskStatus(SubtaskStatus::
						GOING_TO_TARGET.getCommand());
							
		dvrk::Trajectory<dvrk::Pose> to_target = dvrk::TrajectoryFactory::
						linearTrajectoryForSpeed(pose_current,end_target, 
												speed/1000.0, dt);
		checkTrajectory(to_target);
		arm.playTrajectory(to_target);

		
		if (position_type == dvrk_vision::PositionType::DP)
		{
			vision.sendSubtaskStatus(
							SubtaskStatus::
							DP_REACHED.getCommand());

			ROS_INFO_STREAM("DP reached.");
		} 
		else if (position_type == dvrk_vision::PositionType::GOAL)
		{
			vision.sendSubtaskStatus(
							SubtaskStatus::
							GOAL_REACHED.getCommand());
			ROS_INFO_STREAM("Goal reached.");
		}
		else
		{
			throw std::runtime_error("Wrong target type received... ");
		}
	}
}

/**
 * Push tool between tissues in the current direction.
 *
 * @param depth insertion depth in mm
 * @param speed insertion speed in mm/s
 */
void  BluntDissector::toolPushIn(dvrk::PSM &arm,  
			dvrk_vision::VisionConn &vision,
			double depth, double speed /* = 2.0 */)
{
	Eigen::Vector3d v(0,0,1);
	dvrk::Pose p1 = arm.getPoseCurrent();
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
   						
   	arm.playTrajectory(tr); 
}

/**
 * Push tool out of tissues in the current direction.
 *
 * @param depth pull distance in mm
 * @param speed pull speed in mm/s
 */	
void  BluntDissector::toolPullOut(dvrk::PSM &arm, 
			dvrk_vision::VisionConn &vision,
			double depth, double speed /* = 2.0 */)
{
	Eigen::Vector3d v(0,0,1);
	dvrk::Pose p1 = arm.getPoseCurrent();
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
   						
   	arm.playTrajectory(tr); 
}

/**
 * Open tool to separate tissues.
 *
 * @param angle angle of jaws in deg
 * @param speed opening speed in deg/s
 */	
void  BluntDissector::toolOpen(dvrk::PSM &arm, 
			dvrk_vision::VisionConn &vision,
			double angle, double speed /* = 10.0 */)
{
	dvrk::Pose p1 = arm.getPoseCurrent();
	
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
   						
   	arm.playTrajectory(tr); 
}

/**
 * Close tool after separated tissues.
 *
 * @param angle angle of jaws in deg, default is fully closed
 * @param speed closing speed in deg/s
 */	
void  BluntDissector::toolClose(dvrk::PSM &arm, 
				dvrk_vision::VisionConn &vision,
				double angle /* = 0.0 */, 
				double speed /* = 10.0 */)
{
	dvrk::Pose p1 = arm.getPoseCurrent();
	
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
   						
   	arm.playTrajectory(tr); 
}

/**
 * Rotate tool.
 *
 * @param angle angle of rotation
 * @param speed rotation speed in deg/s
 */	
void  BluntDissector::toolRotate(dvrk::PSM &arm, 
			dvrk_vision::VisionConn &vision,
			double angle, double speed /* = 10.0 */)
{
	double angle_rad = (angle/360.0)* M_PI * 2.0;
	double speed_rad = (speed/360.0)* M_PI * 2.0;
		
	double T = std::abs(angle_rad / speed_rad);

	dvrk::Pose p1 = arm.getPoseCurrent();
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
   						
   	arm.playTrajectory(tr); 
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
	checkPose(dissector_arm.getPoseCurrent());
	checkPose(retractor_arm.getPoseCurrent());
}



void  BluntDissector::checkPose(dvrk::Pose p)
{
	Eigen::Vector3d position = p.position;
	
	//TODO
	
	//throw std::runtime_error(errstring.str());
}




}
