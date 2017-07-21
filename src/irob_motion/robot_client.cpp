/*
 *  robot_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-20
 *  
 */

#include "irob_motion/robot_client.hpp"

using namespace ias;


RobotClient::RobotClient(ros::NodeHandle nh, std::string arm_name, double dt): 
			nh(nh), arm_name(arm_name), dt(dt),
			init_arm_ac("init_arm", true),
			reset_pose_ac("reset_pose", true),
			follow_tr_ac("follow_trajectory", true)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    waitForActionServers();
}

RobotClient::~RobotClient()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */

// Read pos and forward
void RobotClient::positionCartesianCurrentCB(
				const irob_autosurg::ToolPoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
    position_cartesian_current_pub.publish(msg);
}

void RobotClient::followTrajectoryDoneCB(
			const actionlib::SimpleClientGoalState& state,
            const irob_autosurg::FollowTrajectoryResultConstPtr& result)
{
	follow_tr_done = true;
}

// TODO remap
void RobotClient::subscribeTopics() 
{                 	            	
   	position_cartesian_current_sub = 
   			nh.subscribe<irob_autosurg::ToolPoseStamped>(
                        "position_cartesian_current_in",
                       	1000, &RobotClient::positionCartesianCurrentCB,this);
}

// TODO remap
void RobotClient::advertiseTopics() 
{
	position_cartesian_current_pub 
				= nh.advertise<irob_autosurg::ToolPoseStamped>(
                    	"position_cartesian_current_out",
                        1000);   
}

void RobotClient::waitForActionServers() 
{
	ROS_INFO_STREAM("Wating for action servers...");
	init_arm_ac.waitForServer();
	reset_pose_ac.waitForServer();
    follow_tr_ac.waitForServer();
    ROS_INFO_STREAM("Action servers started");
}


Pose RobotClient::getPoseCurrent()
{
 	ros::spinOnce();
 	Pose ret(position_cartesian_current);
 	return ret;

}

std::string RobotClient::getName()
{
	return arm_name;
}
   	

// Robot motions

void RobotClient::initArm(bool move_allowed, bool reset_pose)
{
    // Send a goal to the action
  	irob_autosurg::InitArmGoal goal;
 
  	goal.move_allowed = move_allowed;
  	init_arm_ac.sendGoal(goal);
  
  	// Throw exception if no result is received 
  	if(!init_arm_ac.waitForResult(ros::Duration(30.0)))
  		throw std::runtime_error("Arm initialization has timed out");
  	
  	// Pose reset
  	if (reset_pose)
  		resetPose(move_allowed);
}

void RobotClient::resetPose(bool move_allowed)
{
	// Send a goal to the action
  	irob_autosurg::ResetPoseGoal goal;
 
  	goal.move_allowed = move_allowed;
  	reset_pose_ac.sendGoal(goal);
  
  	// Throw exception if no result is received 
  	if(!reset_pose_ac.waitForResult(ros::Duration(30.0)))
  		throw std::runtime_error("Arm pose reset has timed out");
}

/**
 * Open tool to separate tissues.
 *
 * @param angle angle of jaws in deg
 * @param speed opening speed in deg/s
 */	
void RobotClient::moveGripper(double angle, double speed /*=10.0*/)
{
	Pose p1 = getPoseCurrent();
	
	double angle_rad = (std::abs(angle)/360.0)* M_PI * 2.0;
	double speed_rad = (std::abs(speed)/360.0)* M_PI * 2.0;
		
	double T = std::abs((angle_rad-p1.jaw) / speed_rad);
	
		
	Pose p2 = p1;
	p2.jaw = angle_rad;
	
	Trajectory<Pose> tr = TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);						
   						
   	irob_autosurg::FollowTrajectoryGoal goal;
   	tr.copyToRosTrajectory(goal.trajectory);
   	
   	follow_tr_done = false;
   	
  	follow_tr_ac.sendGoal(goal, boost::bind(				
  						&RobotClient::followTrajectoryDoneCB, this, _1, _2));
  	
  	// Not waiting for action finish here, a notification will be received
  	// in followTrajectoryDoneCB
}

void RobotClient::goTo(Pose target, double speed /* = 10.0 */,
			std::vector<Pose> waypoints /* = empty vector */, 
			InterpolationMethod interp_method /* = LINEAR */)
{
	Pose p1 = getPoseCurrent();
	double T;
	Trajectory<Pose> tr;
	
	if (waypoints.empty()) {
	// Go straight to target
	T = std::abs((p1.dist(target)).cartesian / speed);
	tr = TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						target,
   						T, T*0.1, dt);
	
	} else if (interp_method == LINEAR) {
	// Linear trajectory through waypoints
	// TODO
		throw std::runtime_error(
			"Trajectory through waypoints is not implemented yet");		
	} else {
	// Go on Bezier curve through waypoints
	// TODO
		throw std::runtime_error(
			"Trajectory through waypoints is not implemented yet");	
	}
	
	irob_autosurg::FollowTrajectoryGoal goal;
   	tr.copyToRosTrajectory(goal.trajectory);
  	follow_tr_ac.sendGoal(goal);
  	
  	// Throw exception if no result is received 
  	if(!reset_pose_ac.waitForResult(ros::Duration(30.0+T)))
  		throw std::runtime_error("Follow trajectory action has timed out");

}

void RobotClient::moveRelative(Pose p, double speed, CoordFrame cf)
{
	// TODO
	throw std::runtime_error(
			"Tool rotation is not implemented yet");	
}

void RobotClient::moveRelative(Eigen::Vector3d v,  double speed, CoordFrame cf)
{
	Pose p1 = getPoseCurrent();
	Eigen::Vector3d v_rot;
	
	switch (cf) {
		case WCS:
			v_rot = v;
			break;
			
		case TCPF:
			Eigen::Matrix3d R = p1.orientation.toRotationMatrix();
			v_rot = R*v;
			
			break;
	}
	
	Pose p2 = p1 + v_rot;
	
	double T = std::abs(v_rot.norm() / speed);
	Trajectory<Pose> tr = TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);
	
	irob_autosurg::FollowTrajectoryGoal goal;
   	tr.copyToRosTrajectory(goal.trajectory);
  	follow_tr_ac.sendGoal(goal);
  	
  	// Throw exception if no result is received 
  	if(!reset_pose_ac.waitForResult(ros::Duration(30.0+T)))
  		throw std::runtime_error("Follow trajectory action has timed out");
	
}

void RobotClient::moveRelative(Eigen::Quaternion<double> q,
									double speed, CoordFrame cf)
{
	// TODO
	throw std::runtime_error(
			"Tool rotation is not implemented yet");	
}

bool RobotClient::isFollowTrajectoryDone()
{
	ros::spinOnce();
	return follow_tr_done;
}





























