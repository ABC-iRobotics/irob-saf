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
			nh(nh), arm_name(arm_name), dt(dt)
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
				const geometry_msgs::PoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
    position_cartesian_current_pub.publish(msg);
}

// TODO remap
void RobotClient::subscribeTopics() 
{                 	            	
   	position_cartesian_current_sub = nh.subscribe<geometry_msgs::PoseStamped>(
                        "position_cartesian_current_in",
                       	1000, &Arm::positionCartesianCurrentCB,this);
}

// TODO remap
void RobotClient::advertiseTopics() 
{
	position_cartesian_current_pub 
				= nh.advertise<irob_autosurg::ToolPoseStamped>(
                    	"position_cartesian_current_out",
                        1000);   
}

void RobotClient::startActionServers() 
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
 	Pose ret(position_cartesian_current, 0.0);
 	return ret;

}

// Robot motions

void RobotClient::initArm(bool move_allowed, bool reset_pose)
{
    // Send a goal to the action
  	irob_autosurg::InitArmGoal goal;
 
  	goal.move_allowed = move_allowed;
  	init_arm_ac.sendGoal(goal);
  
  	// Throw exception if no result is received 
  	if(!init_arm_ac.waitForResult(ros::Duration(30.0))))
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
  	if(!reset_pose_ac.waitForResult(ros::Duration(30.0))))
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
	
		
	dvrk::Pose p2 = p1;
	p2.jaw = angle_rad;
	
	dvrk::Trajectory<dvrk::Pose> tr = dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);						
   						
   	irob_autosurg::FollowTrajectoryGoal goal;
   	tr.copyToRosTrajectory(goal.trajectory);
  	follow_tr_ac.sendGoal(goal);
  	
  	// Throw exception if no result is received 
  	if(!reset_pose_ac.waitForResult(ros::Duration(30.0+T))))
  		throw std::runtime_error("Follow trajectory action has timed out");
   	
}

void RobotClient::goTo(Pose target, double speed /* = 10.0 */,
			vector<Pose> waypoints /* = empty vector */, 
			InterpolatonMethod interp_method /* = InterpolationMethod.LINEAR */)
{
	Pose p1 = getPoseCurrent();
	double T;
	dvrk::Trajectory<dvrk::Pose> tr;
	
	if (waypoints.empty()) {
	// Go straight to target
	T = std::abs((p1.dist(target)).cartesian / speed_rad);
	tr = dvrk::TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						T, T*0.1, dt);
	
	} else if (interp_method == InterpolationMethod.LINEAR) {
	// Linear trajectory through waypoints
	
	} else {
	// Go on Bezier curve through waypoints
	
	}

}

void RobotClient::moveRelative(Pose, CoordFrame)
{

}

void RobotClient::moveRelative(Eigen::Vector3d, CoordFrame)
{

}

void RobotClient::moveRelative(Eigen::Quaternion<double>, CoordFrame)
{

}





























