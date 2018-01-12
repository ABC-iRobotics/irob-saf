/*
 * 	gesture_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-24
 *  
 */

#include <irob_motion/gesture_client.hpp>

namespace ias {

const double GestureClient::DEFAULT_SPEED_CARTESIAN = 30.0;	// mm/s
const double GestureClient::DEFAULT_SPEED_JAW = 10.0;		// deg/s


GestureClient::GestureClient(ros::NodeHandle nh, std::string arm_name): 
			nh(nh), arm_name(arm_name),
			ac("gesture/"+arm_name, true)
{

	// Subscribe and advertise topics
	
	subscribeTopics();
    advertiseTopics();
    waitForActionServer();
}

GestureClient::~GestureClient()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */

// Read pos 
void GestureClient::positionCartesianCurrentCB(
				const irob_msgs::ToolPoseStampedConstPtr& msg) 
{
    position_cartesian_current = *msg;
    position_cartesian_current_pub.publish(msg);
}

void GestureClient::instrumentInfoCB(
				const irob_msgs::InstrumentInfoConstPtr& msg) 
{
    instrument_info = *msg;
    instrument_info_pub.publish(msg);
}


void GestureClient::subscribeTopics() 
{                 	            	
   	position_cartesian_current_sub = 
   			nh.subscribe<irob_msgs::ToolPoseStamped>(
                        "gesture/"+arm_name+"/position_cartesian_current_cf",
                       	1000, &GestureClient::positionCartesianCurrentCB,this);
                    
    instrument_info_sub = 
   			nh.subscribe<irob_msgs::InstrumentInfo>(
                        "gesture/"+arm_name+"/instrument_info",
                       	1000, &GestureClient::instrumentInfoCB,this);
}


void GestureClient::advertiseTopics() 
{
	position_cartesian_current_pub 
				= nh.advertise<irob_msgs::ToolPoseStamped>(
                    	"maneuver/"+arm_name+"/position_cartesian_current_cf",
                        1000);   

    instrument_info_pub 
				= nh.advertise<irob_msgs::InstrumentInfo>(
                    	"maneuver/"+arm_name+"/instrument_info",
                        1000);   
}


void GestureClient::waitForActionServer() 
{
	ROS_INFO_STREAM("Wating for action server...");
	ac.waitForServer();
    ROS_INFO_STREAM("Action servers started");
}


Pose GestureClient::getPoseCurrent()
{
 	while (position_cartesian_current.header.seq == 0)
	{
 		ros::spinOnce();
 		ros::Duration(0.05).sleep();
 	}
 	Pose ret(position_cartesian_current);
 	return ret;

}


irob_msgs::InstrumentInfo GestureClient::getInstrumentInfo()
{
	while (instrument_info.name.empty())
	{
 		ros::spinOnce();
 		ros::Duration(0.05).sleep();
 	}
 	irob_msgs::InstrumentInfo ret(instrument_info);
 	return ret;
}

std::string GestureClient::getName()
{
	return arm_name;
}
   	

// Robot motions

void GestureClient::stop()
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::STOP;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}

void GestureClient::nav_to_pos(Pose target,
					double speed_cartesian/* = DEFAULT_SPEED_CARTESIAN */,
					std::vector<Pose> waypoints /* = empty */, 
					InterpolationMethod interp_method /* = LINEAR */)
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::NAV_TO_POS;
  	
  	for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
    
    if (interp_method == InterpolationMethod::BEZIER)
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_BEZIER;
    else
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_LINEAR;
 	
 	goal.target = target.toRosPose();
 	
 	goal.speed_cartesian = speed_cartesian;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}

void GestureClient::grasp(Pose target, Pose approach_pose,
					double target_diameter,
					double compression_rate,
					double speed_cartesian /* = DEFAULT_SPEED_CARTESIAN */,
					double speed_jaw /* = DEFAULT_SPEED_JAW */,
					std::vector<Pose> waypoints /* = empty */, 
					InterpolationMethod interp_method /* = LINEAR */)
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::GRASP;
 	
 	goal.target = target.toRosPose();
	goal.approach_pose = approach_pose.toRosPose();
	
    for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
    
    if (interp_method == InterpolationMethod::BEZIER)
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_BEZIER;
    else
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_LINEAR;
 	
  	goal.target_diameter = target_diameter;
  	goal.compression_rate = compression_rate;
  	goal.speed_cartesian = speed_cartesian;
  	goal.speed_jaw = speed_jaw;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


void GestureClient::cut(Pose target, Pose approach_pose,
					double target_diameter,
					double speed_cartesian /* = DEFAULT_SPEED_CARTESIAN */,
					double speed_jaw /* = DEFAULT_SPEED_JAW */,
					std::vector<Pose> waypoints /* = empty */, 
					InterpolationMethod interp_method /* = LINEAR */)
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::CUT;
 	
  	goal.target = target.toRosPose();
	goal.approach_pose = approach_pose.toRosPose();
	
    for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
    
    if (interp_method == InterpolationMethod::BEZIER)
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_BEZIER;
    else
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_LINEAR;
 	
  	goal.target_diameter = target_diameter;
  	goal.speed_cartesian = speed_cartesian;
  	goal.speed_jaw = speed_jaw;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


void GestureClient::release(Pose approach_pose,	double target_diameter,
			double speed_cartesian /* = DEFAULT_SPEED_CARTESIAN */,
			double speed_jaw /* = DEFAULT_SPEED_JAW */)
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::RELEASE;
 	
	goal.approach_pose = approach_pose.toRosPose();
	
 
  	goal.target_diameter = target_diameter;
  	goal.speed_cartesian = speed_cartesian;
  	goal.speed_jaw = speed_jaw;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


void GestureClient::place(Pose target, Pose approach_pose,
				double speed_cartesian /* = DEFAULT_SPEED_CARTESIAN */,
				std::vector<Pose> waypoints /* = empty */, 
				InterpolationMethod interp_method /* = LINEAR */)
{
    // Send a goal to the action
  	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::PLACE;
 	
   	goal.target = target.toRosPose();
	goal.approach_pose = approach_pose.toRosPose();
	
    for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
    
    if (interp_method == InterpolationMethod::BEZIER)
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_BEZIER;
    else
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_LINEAR;
 	
  	goal.speed_cartesian = speed_cartesian;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


void GestureClient::push(Pose target, Pose approach_pose, 
				Eigen::Vector3d displacement,
				double speed_cartesian /* = DEFAULT_SPEED_CARTESIAN */,
				double speed_jaw /* = DEFAULT_SPEED_JAW */,
				std::vector<Pose> waypoints /* = empty */, 
				InterpolationMethod interp_method /* = LINEAR */)
{
	// Send a goal to the action
	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::PUSH;
 
  	goal.target = target.toRosPose();
	goal.approach_pose = approach_pose.toRosPose();
	
	geometry_msgs::Point displacement_ros;
   	displacement_ros.x = displacement.x();
   	displacement_ros.y = displacement.y();
   	displacement_ros.z = displacement.z();
   	goal.displacement = displacement_ros;
   	
    for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
    
    if (interp_method == InterpolationMethod::BEZIER)
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_BEZIER;
    else
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_LINEAR;
 	
  	goal.speed_cartesian = speed_cartesian;
  	goal.speed_jaw = speed_jaw;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


void GestureClient::dissect(Pose target, Pose approach_pose, 
			Eigen::Vector3d displacement,
			double target_diameter,
			double speed_cartesian /* = DEFAULT_SPEED_CARTESIAN */,
			double speed_jaw /* = DEFAULT_SPEED_JAW */,
			std::vector<Pose> waypoints /* = empty */, 
			InterpolationMethod interp_method /* = LINEAR */)
{
	// Send a goal to the action
	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::DISSECT;
 
  	goal.target = target.toRosPose();
	goal.approach_pose = approach_pose.toRosPose();
	
	geometry_msgs::Point displacement_ros;
   	displacement_ros.x = displacement.x();
   	displacement_ros.y = displacement.y();
   	displacement_ros.z = displacement.z();
   	goal.displacement = displacement_ros;
   	
    for (Pose p : waypoints)
    	goal.waypoints.push_back(p.toRosPose());
    
    if (interp_method == InterpolationMethod::BEZIER)
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_BEZIER;
    else
    	goal.interpolation = irob_msgs::GestureGoal::INTERPOLATION_LINEAR;
 	
 	goal.target_diameter = target_diameter;
  	goal.speed_cartesian = speed_cartesian;
  	goal.speed_jaw = speed_jaw;
  	
  	ac.sendGoal(goal);
  	
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


void GestureClient::manipulate(Eigen::Vector3d displacement,
					double speed_cartesian /* = DEFAULT_SPEED_CARTESIAN */)
{
	// Send a goal to the action
	irob_msgs::GestureGoal goal;
 
 	goal.action = irob_msgs::GestureGoal::MANIPULATE;
	
	geometry_msgs::Point displacement_ros;
   	displacement_ros.x = displacement.x();
   	displacement_ros.y = displacement.y();
   	displacement_ros.z = displacement.z();
   	goal.displacement = displacement_ros;
   	
  	goal.speed_cartesian = speed_cartesian;
  	
  	ac.sendGoal(goal);
  	// Not waiting for action finish here, a notification will be received
  	// in gestureDoneCB
}


bool GestureClient::isGestureDone(bool spin /* = true */)
{
	return ac.isDone(spin);
}

actionlib::SimpleClientGoalState GestureClient::getState()
{
	return ac.getState();
}

irob_msgs::GestureFeedback GestureClient::getFeedback(bool spin /* = true */)
{
	return ac.getFeedback(spin);
}

irob_msgs::GestureResult GestureClient::getResult(bool spin /* = true */)
{
	return ac.getResult(spin);
}

}






















