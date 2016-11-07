/*
 * dvrk_arm.cpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#include "dvrk_arm.hpp"
#include <numeric>
#include <chrono>


const std::string DVRKArm::HOME_CMD
                    = "Home";
const std::string DVRKArm::HOME_DONE
                    = "DVRK_READY";
const std::string DVRKArm::STATE_POSITION_JOINT
                    = "DVRK_POSITION_JOINT";
                   // = "DVRK_POSITION_GOAL_JOINT";
const std::string DVRKArm::STATE_POSITION_CARTESIAN
                    ="DVRK_POSITION_CARTESIAN";
                   // ="DVRK_POSITION_GOAL_CARTESIAN";


DVRKArm::DVRKArm(ros::NodeHandle nh, DVRKArmTypes arm_typ): nh(nh), arm_typ(arm_typ) {
	// TODO Auto-generated constructor stub
}

DVRKArm::~DVRKArm() {
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void DVRKArm::robotStateCB(const std_msgs::String msg) {
    robot_state = msg;
    //ROS_INFO("Robot state: %s", msg.data.c_str());
}

void DVRKArm::stateJointCurrentCB(const sensor_msgs::JointStateConstPtr& msg) {
    position_joint = *msg;
    //ROS_INFO("Joint state:\n");
   // for (int i = 0; i < arm_typ.getDof(); i++)
        //ROS_INFO("\tJoint %d:\t%f\n", i, position_joint.position[i]);
}

void DVRKArm::positionCartesianCurrentCB(const geometry_msgs::PoseStampedConstPtr& msg) {
     position_cartesian_current = *msg;
     //ROS_INFO("State cartesian:\n");
         //ROS_INFO("X: %f\tY: %f\tZ: %f\n",position_cartesian_current.pose.position.x, position_cartesian_current.pose.position.y, position_cartesian_current.pose.position.z);
}

bool DVRKArm::subscribe(DVRKArmTopics topic) {
    if(topic == DVRKArmTopics::GET_ROBOT_STATE)
    {
        robot_state_sub = nh.subscribe<std_msgs::String>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::robotStateCB,this);
    }
    else if( topic == DVRKArmTopics::GET_STATE_JOINT_CURRENT)
    {
        state_joint_current_sub = nh.subscribe<sensor_msgs::JointState>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::stateJointCurrentCB,this);
    }
    else if( topic == DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT)
    {
        position_cartesian_current_sub = nh.subscribe<geometry_msgs::PoseStamped>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::positionCartesianCurrentCB,this);
    }
    else
    {
        ROS_INFO("Subscribing to invalid topic %s", topic.getFullName(arm_typ).c_str());
        return false;
    }

    ROS_INFO("Subscribed to topic %s", topic.getFullName(arm_typ).c_str());
    return true;
}

bool DVRKArm::advertise(DVRKArmTopics topic) {
    if(topic == DVRKArmTopics::SET_ROBOT_STATE)
    {
        robot_state_pub = nh.advertise<std_msgs::String>(
                                    topic.getFullName(arm_typ), 1000);
    }
    else if(topic == DVRKArmTopics::SET_POSITION_JOINT)
    {
        position_joint_pub = nh.advertise<sensor_msgs::JointState>(
                                   topic.getFullName(arm_typ), 1000);
    }
    else if(topic == DVRKArmTopics::SET_POSITION_CARTESIAN)
    {
        position_cartesian_pub = nh.advertise<geometry_msgs::Pose>(
                                   topic.getFullName(arm_typ), 1000);
    }
    else if(topic == DVRKArmTopics::SET_POSITION_JAW)
    {
        position_jaw_pub = nh.advertise<std_msgs::Float32>(
                                   topic.getFullName(arm_typ), 1000);
    }
    else {
         ROS_INFO("Advertising invalid topic %s", topic.getFullName(arm_typ).c_str());
         return false;
    }
    ROS_INFO("Advertised topic %s", topic.getFullName(arm_typ).c_str());
    return true;
}

/*
 * DVRK actions
 */
double DVRKArm::getJointStateCurrent(int index)
{
	ros::spinOnce();
 	return position_joint.position[index];
}
 
Vector3D DVRKArm::getPositionCartesianCurrent()
{
 	ros::spinOnce();
 	Vector3D ret(position_cartesian_current);
    return ret;
}

Quaternion DVRKArm::getOrientationCartesianCurrent()
{
	ros::spinOnce();
 	Quaternion ret(position_cartesian_current.pose.orientation.x,
 		position_cartesian_current.pose.orientation.y, 
 		position_cartesian_current.pose.orientation.z, 
 		position_cartesian_current.pose.orientation.w);
    return ret;
}

Pose DVRKArm::getPoseCurrent()
{
 	ros::spinOnce();
 	Pose ret(position_cartesian_current, position_joint.position[6]);
    return ret;
}


bool DVRKArm::home() {
    while (true) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << HOME_CMD;
        msg.data = ss.str();
        robot_state_pub.publish(msg);
        ros::Duration(0.5).sleep();

        if (robot_state.data == HOME_DONE) {
            ROS_INFO("State set to Home");
            return true;
        } else {
            //ROS_INFO("State set to %s\n", robot_state.data.c_str());
        }

        ros::spinOnce();
    }
    return false;
}

bool DVRKArm::setRobotState(std::string state)
{
    while (true) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << state;
        msg.data = ss.str();
        robot_state_pub.publish(msg);
        ros::Duration(0.5).sleep();

        if (robot_state.data == state) {
            ROS_INFO("State set to %s", state.c_str());
            return true;
        } else {
            ROS_INFO("State set to %s", robot_state.data.c_str());
        }

        ros::spinOnce();
    }
    return false;
}

void DVRKArm::moveJointRelative(int joint_idx, double movement)
{
    ros::spinOnce();
    //ROS_INFO("Position not received!\n");
    sensor_msgs::JointState new_position_joint = position_joint;
    // TODO safety
    new_position_joint.position[joint_idx] += movement;
    position_joint_pub.publish(new_position_joint);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();
}

void DVRKArm::moveJointAbsolute(int joint_idx, double pos)
{
    ros::spinOnce();
    //ROS_INFO("Position not received!\n");
    sensor_msgs::JointState new_position_joint = position_joint;
    // TODO safety
    new_position_joint.position[joint_idx] = pos;
    position_joint_pub.publish(new_position_joint);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();
}

void DVRKArm::moveCartesianRelative(Vector3D movement)
{
    ros::spinOnce();
    geometry_msgs::Pose new_position_cartesian = position_cartesian_current.pose;
    // TODO safety

    new_position_cartesian.position.x += movement.x;
    new_position_cartesian.position.y += movement.y;
    new_position_cartesian.position.z += movement.z;

    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void DVRKArm::moveCartesianAbsolute(Vector3D position)
{
    ros::spinOnce();
    geometry_msgs::Pose new_position_cartesian = position_cartesian_current.pose;
    // TODO safety

    new_position_cartesian.position.x = position.x;
    new_position_cartesian.position.y = position.y;
    new_position_cartesian.position.z = position.z;

    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void DVRKArm::moveCartesianAbsolute(Quaternion orientation)
{
    ros::spinOnce();
    geometry_msgs::Pose new_position_cartesian = position_cartesian_current.pose;
    // TODO safety

    new_position_cartesian.orientation.v.x = orientation.x;
    new_position_cartesian.orientation.v.y = orientation.y;
    new_position_cartesian.orientation.v.z = orientation.z;
    new_position_cartesian.orientation.w = orientation.w;

    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void DVRKArm::moveCartesianAbsolute(Vector3D position, Quaternion orientation)
{
    ros::spinOnce();
    geometry_msgs::Pose new_position_cartesian = position_cartesian_current.pose;
    // TODO safety

    new_position_cartesian.position.x = position.x;
    new_position_cartesian.position.y = position.y;
    new_position_cartesian.position.z = position.z;
    
    new_position_cartesian.orientation.x = orientation.x;
    new_position_cartesian.orientation.y = orientation.y;
    new_position_cartesian.orientation.z = orientation.z;
    new_position_cartesian.orientation.w = orientation.w;

    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void DVRKArm::moveCartesianAbsolute(Pose pose)
{
    ros::spinOnce();
    geometry_msgs::Pose new_position_cartesian = position_cartesian_current.pose;
    std_msgs::Float32 new_position_jaw;
    // TODO safety

    new_position_cartesian.position.x = pose.position.x;
    new_position_cartesian.position.y = pose.position.y;
    new_position_cartesian.position.z = pose.position.z;
    
    new_position_cartesian.orientation.x = pose.orientation.x;
    new_position_cartesian.orientation.y = pose.orientation.y;
    new_position_cartesian.orientation.z = pose.orientation.z;
    new_position_cartesian.orientation.w = pose.orientation.w;
    
    new_position_jaw.data = pose.jaw;

    position_cartesian_pub.publish(new_position_cartesian);
    position_jaw_pub.publish(new_position_jaw);
    
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

/*
 * Trajectories
 */
void DVRKArm::playTrajectory(Trajectory<Vector3D>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	//int cnt = 0;
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		//auto start = std::chrono::high_resolution_clock::now();
		
		moveCartesianAbsolute(tr[i]);
		
		loop_rate.sleep();
		/*
		std::chrono::duration<double> elapsed =
	 			std::chrono::high_resolution_clock::now()-start;
	 	cnt ++;
	 	if (cnt >= 10)
	 	{
			ROS_INFO("Time elapsed: %f us", (elapsed*1000000.0));
			cnt = 0;
		}*/
	}

}

void DVRKArm::playTrajectory(Trajectory<Quaternion>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(tr[i]);
		loop_rate.sleep();
	}
}

void DVRKArm::playTrajectory(Trajectory<Vector3D>& p, Trajectory<Quaternion>& o)
{
	ros::Rate loop_rate(1.0/p.dt);
	for (int i = 0; i < p.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(p[i], o[i]);
		
		//TODO p.dt != o.dt
		loop_rate.sleep();
	}
}

void DVRKArm::playTrajectory(Trajectory<Pose>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(tr[i]);
		
		loop_rate.sleep();
	}
}

void DVRKArm::playTrajectory(int jointIndex, Trajectory<double>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveJointAbsolute(jointIndex, tr[i]);
		loop_rate.sleep();
	}
}

void DVRKArm::recordTrajectory(Trajectory<Vector3D>& tr) 
{
	ros::Rate loop_rate(1.0/tr.dt);
	// Skip invalid points
	while (ros::ok() && getPositionCartesianCurrent().length() < 0.001)
  	{
  		loop_rate.sleep();
  	}
	while(ros::ok())
	{
		tr.addPoint(getPositionCartesianCurrent());
		loop_rate.sleep();
	}
}

void DVRKArm::recordTrajectory(Trajectory<Pose>& tr) 
{
	ros::Rate loop_rate(1.0/tr.dt);
	// Skip invalid points
	while (ros::ok() && getPositionCartesianCurrent().length() < 0.001)
  	{
  		loop_rate.sleep();
  	}
	while(ros::ok())
	{
		tr.addPoint(getPoseCurrent());
		loop_rate.sleep();
	}
}




