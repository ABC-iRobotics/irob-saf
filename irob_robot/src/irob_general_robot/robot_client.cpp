/*
 *  robot_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-20
 *  
 */

#include <irob_general_robot/robot_client.hpp>

namespace saf {


RobotClient::RobotClient(ros::NodeHandle nh, std::string arm_name, double dt): 
			nh(nh), arm_name(arm_name), dt(dt),
			ac("robot/"+arm_name+"/robot_action", true)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    waitForActionServer();
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
				const irob_msgs::ToolPoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
    position_cartesian_current_pub.publish(msg);
}

void RobotClient::instrumentInfoCB(
				const irob_msgs::InstrumentInfoConstPtr& msg) 
{
    instrument_info = *msg;
    instrument_info_pub.publish(msg);
}

void RobotClient::subscribeTopics() 
{                 	            	
   	position_cartesian_current_sub = 
   			nh.subscribe<irob_msgs::ToolPoseStamped>(
                        "robot/"+arm_name+"/position_cartesian_current_cf",
                       	1000, &RobotClient::positionCartesianCurrentCB,this);
                       	
    instrument_info_sub = 
   			nh.subscribe<irob_msgs::InstrumentInfo>(
                        "robot/"+arm_name+"/instrument_info",
                       	1000, &RobotClient::instrumentInfoCB,this);
}

void RobotClient::advertiseTopics() 
{
	position_cartesian_current_pub 
				= nh.advertise<irob_msgs::ToolPoseStamped>(
                    	"gesture/"+arm_name+"/position_cartesian_current_cf",
                        1000); 
    
    instrument_info_pub 
				= nh.advertise<irob_msgs::InstrumentInfo>(
                    	"gesture/"+arm_name+"/instrument_info",
                        1000);   
}

void RobotClient::waitForActionServer() 
{
	ROS_INFO_STREAM("Wating for action server...");
	ac.waitForServer();
    ROS_INFO_STREAM("Action server started");
}


Pose RobotClient::getPoseCurrent()
{
	while (position_cartesian_current.header.seq == 0)
	{
 		ros::spinOnce();
 		ros::Duration(0.05).sleep();
 	}
 	Pose ret(position_cartesian_current);
 	return ret;
}

irob_msgs::InstrumentInfo RobotClient::getInstrumentInfo()
{
	while (instrument_info.name.empty())
	{
 		ros::spinOnce();
 		ros::Duration(0.05).sleep();
 	}
 	irob_msgs::InstrumentInfo ret(instrument_info);
 	return ret;
}

std::string RobotClient::getName()
{
	return arm_name;
}
   	

// Robot motions
void RobotClient::initArm(bool move_allowed)
{
    // Send a goal to the action
  	irob_msgs::RobotGoal goal;
  	goal.action = irob_msgs::RobotGoal::INIT_ARM;
 
  	goal.move_allowed = move_allowed;
  	ac.sendGoal(goal);  	
}

void RobotClient::resetPose(bool move_allowed)
{
	// Send a goal to the action
  	irob_msgs::RobotGoal goal;
  	goal.action = irob_msgs::RobotGoal::RESET_POSE;
 
  	goal.move_allowed = move_allowed;
  	ac.sendGoal(goal);
}

void RobotClient::stop()
{
	// Send a goal to the action
  	irob_msgs::RobotGoal goal;
  	goal.action = irob_msgs::RobotGoal::STOP;
 
  	goal.move_allowed = false;
  	ac.sendGoal(goal);
}

/**
 * Move tool gripper.
 *
 * @param angle angle of jaws in deg
 * @param speed opening speed in deg/s
 */	
void RobotClient::moveJaws(double angle, double speed /*=10.0*/)
{
	Pose p1 = getPoseCurrent();
	
	double angle_rad = degToRad(std::abs(angle));
	double speed_rad = degToRad(std::abs(speed));	
		
	Pose p2 = p1;
	p2.jaw = angle_rad;
	
	Trajectory<Pose> tr = TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						speed_rad, speed_rad * 10.0, dt);					
   						
   	irob_msgs::RobotGoal goal;
  	goal.action = irob_msgs::RobotGoal::FOLLOW_TRAJECTORY;
  	goal.move_allowed = true;
  	
   	tr.copyToRosTrajectory(goal.trajectory);
   	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in actionDoneCB
}

/**
 * 
 */
void RobotClient::moveTool(Pose target, double speed /* = 10.0 */,
			std::vector<Pose> waypoints /* = empty vector */, 
			InterpolationMethod interp_method /* = LINEAR */)
{
	Pose p1 = getPoseCurrent();
	Trajectory<Pose> tr;
	
	// Jaw cannot be changed!
	target.jaw = p1.jaw;
  for (Pose &p : waypoints)
		p.jaw = p1.jaw;
	
	if (waypoints.empty()) {
	// Go straight to target

		tr = TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						target,
   						speed, speed * 10.0, dt);
	
	} else if (interp_method == LINEAR) {
	// Linear trajectory through waypoints
			// TODO issue with speed...
		tr = TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						waypoints,
   						target,
   						speed, speed * 10.0, dt);	
	} else {
	// Go on Bezier curve through waypoints
	// TODO
		throw std::runtime_error(
			"Trajectory through waypoints is not implemented yet");	
	}
	
	irob_msgs::RobotGoal goal;
  	goal.action = irob_msgs::RobotGoal::FOLLOW_TRAJECTORY;
  	goal.move_allowed = true;
   	tr.copyToRosTrajectory(goal.trajectory);
  	ac.sendGoal(goal);

}

/**
 * Move relative in TCPF or WCS
 
void RobotClient::moveRelative(Eigen::Vector3d v,  double speed, CoordFrame cf)
{
	Pose p1 = getPoseCurrent();
	Eigen::Vector3d v_rot;
	
	// Rotate v to TCP or WCS
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
	
	Trajectory<Pose> tr = TrajectoryFactory::
   				linearTrajectoryWithSmoothAcceleration(
   						p1, 
   						p2,
   						speed, speed * 10.0, dt);
	
	irob_msgs::FollowTrajectoryGoal goal;
   	tr.copyToRosTrajectory(goal.trajectory);
  	follow_tr_ac.sendGoal(goal);
	
}

void RobotClient::moveRelative(Eigen::Quaternion<double> q,
									double speed, CoordFrame cf)
{
	// TODO
	throw std::runtime_error(
			"Tool rotation is not implemented yet");	
}
*/

bool RobotClient::isActionDone(bool spin /* = true */)
{
	return ac.isDone(spin);
}

actionlib::SimpleClientGoalState RobotClient::getState()
{
	return ac.getState();
}

irob_msgs::RobotFeedback RobotClient::getFeedback(bool spin /* = true */)
{
	return ac.getFeedback(spin);
}

irob_msgs::RobotResult RobotClient::getResult(bool spin /* = true */)
{
	return ac.getResult(spin);
}








}



















