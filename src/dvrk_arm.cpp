/*
 * dvrk_arm.cpp
 *
 *  Created on: 2016. okt. 10.
 *      Author: tamas
 */

#include "dvrk_arm.hpp"


const std::string DVRKArm::HOME_CMD
                    = "Home";
const std::string DVRKArm::HOME_DONE
                    = "DVRK_READY";
const std::string DVRKArm::STATE_POSITION_JOINT
                    = "DVRK_POSITION_JOINT";
const std::string DVRKArm::STATE_POSITION_CARTESIAN
                    ="DVRK_POSITION_GOAL_CARTESIAN";


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
     ROS_INFO("State cartesian:\n");
         ROS_INFO("X: %f\tY: %f\tZ: %f\n",position_cartesian_current.pose.position.x, position_cartesian_current.pose.position.y, position_cartesian_current.pose.position.z);
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
        ROS_INFO("Subscribing to invalid topic %s\n", topic.getFullName(arm_typ).c_str());
        return false;
    }

    ROS_INFO("Subscribed to topic %s\n", topic.getFullName(arm_typ).c_str());
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
    else {
         ROS_INFO("Advertising invalid topic %s\n", topic.getFullName(arm_typ).c_str());
         return false;
    }
    ROS_INFO("Advertised topic %s\n", topic.getFullName(arm_typ).c_str());
    return true;
}

/*
 * DVRK actions
 */

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
            ROS_INFO("State set to %s\n", robot_state.data.c_str());
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
    ros::Duration(0.1).sleep();
}

void DVRKArm::moveCartesianRelative(std::vector<double> movement)
{
    ros::spinOnce();
    geometry_msgs::Pose new_position_cartesian = position_cartesian_current.pose;
    // TODO safety

    new_position_cartesian.position.x += movement[0];
    new_position_cartesian.position.y += movement[1];
    new_position_cartesian.position.z += movement[2];

    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

}

void DVRKArm::moveCartesianAbsolute(std::vector<double> position)
{
    ros::spinOnce();
    geometry_msgs::Pose new_position_cartesian = position_cartesian_current.pose;
    // TODO safety

    new_position_cartesian.position.x = position[0];
    new_position_cartesian.position.y = position[1];
    new_position_cartesian.position.z = position[2];

    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

}

