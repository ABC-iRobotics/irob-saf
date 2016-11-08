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


DVRKArm::DVRKArm(ros::NodeHandle nh, DVRKArmTypes arm_typ, bool isActive): 
							nh(nh), arm_typ(arm_typ)
{

	// Subscribe and advertise topics
	subscribe(DVRKArmTopics::GET_ROBOT_STATE);
    subscribe(DVRKArmTopics::GET_STATE_JOINT_CURRENT);
    subscribe(DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT);
    subscribe(DVRKArmTopics::GET_ERROR);
    subscribe(DVRKArmTopics::GET_WARNING);
    
    if (isActive == ACTIVE)
    {
    	advertise(DVRKArmTopics::SET_ROBOT_STATE);
    	advertise(DVRKArmTopics::SET_POSITION_JOINT);
    	advertise(DVRKArmTopics::SET_POSITION_CARTESIAN);
    	
    	if (arm_typ == DVRKArmTypes::PSM1 || arm_typ == DVRKArmTypes::PSM2)
    		advertise(DVRKArmTopics::SET_POSITION_JAW);
    }
    
}

DVRKArm::~DVRKArm()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void DVRKArm::robotStateCB(const std_msgs::String msg)
{
    robot_state = msg;
}

void DVRKArm::stateJointCurrentCB(const sensor_msgs::JointStateConstPtr& msg) 
{
    position_joint = *msg;
}

void DVRKArm::positionCartesianCurrentCB(
				const geometry_msgs::PoseStampedConstPtr& msg) 
{
     position_cartesian_current = *msg;
}

void DVRKArm::errorCB(const std_msgs::String msg) 
{
     error = msg;
}

void DVRKArm::warningCB(const std_msgs::String msg) 
{
     warning = msg;
}

bool DVRKArm::subscribe(DVRKArmTopics topic) 
{
    if(topic == DVRKArmTopics::GET_ROBOT_STATE)
    {
        robot_state_sub = nh.subscribe<std_msgs::String>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::robotStateCB,this);
    }
    else if( topic == DVRKArmTopics::GET_STATE_JOINT_CURRENT)
    {
        state_joint_current_sub 
        				= nh.subscribe<sensor_msgs::JointState>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::stateJointCurrentCB,this);
    }
    else if( topic == DVRKArmTopics::GET_POSITION_CARTESIAN_CURRENT)
    {
        position_cartesian_current_sub 
        				= nh.subscribe<geometry_msgs::PoseStamped>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::positionCartesianCurrentCB,this);
    }
    else if( topic == DVRKArmTopics::GET_ERROR)
    {
       	error_sub 	= nh.subscribe<std_msgs::String>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::errorCB,this);
    }
    else if( topic == DVRKArmTopics::GET_WARNING)
    {
       	warning_sub 	= nh.subscribe<std_msgs::String>(
                        topic.getFullName(arm_typ), 1000,
                        &DVRKArm::warningCB,this);
    }
    else
    {
        ROS_WARN_STREAM("Subscribing to invalid topic " 
        	<<  topic.getFullName(arm_typ));
        return false;
    }

    ROS_INFO_STREAM("Subscribed to topic " 
    	<< topic.getFullName(arm_typ));
    return true;
}

bool DVRKArm::advertise(DVRKArmTopics topic) 
{
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

std::vector<double> DVRKArm::getJointStateCurrent()
{
	ros::spinOnce();
	std::vector<double> ret(arm_typ.dof);
	
		ret = position_joint.position;
 	return ret;
}
 
Eigen::Vector3d DVRKArm::getPositionCartesianCurrent()
{
 	ros::spinOnce();
 	Eigen::Vector3d ret(position_cartesian_current.pose.position.x,
 						position_cartesian_current.pose.position.y,
 						position_cartesian_current.pose.position.z);
    return ret;
}

Eigen::Quaternion<double> DVRKArm::getOrientationCartesianCurrent()
{
	ros::spinOnce();
 	Eigen::Quaternion<double> ret(position_cartesian_current.pose.orientation.x,
 		position_cartesian_current.pose.orientation.y, 
 		position_cartesian_current.pose.orientation.z, 
 		position_cartesian_current.pose.orientation.w);
    return ret;
}

Pose DVRKArm::getPoseCurrent()
{
 	ros::spinOnce();
 	if (arm_typ == DVRKArmTypes::PSM1 || arm_typ == DVRKArmTypes::PSM2)
 	{
 		Pose ret(position_cartesian_current, position_joint.position[6]);
 		return ret;
 	}
 	else
 	{
 		Pose ret(position_cartesian_current, 0.0);
 		return ret;
 	}
    
}


bool DVRKArm::home()
{
    while (true) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << HOME_CMD;
        msg.data = ss.str();
        robot_state_pub.publish(msg);
        ros::Duration(0.5).sleep();

        if (robot_state.data == HOME_DONE) {
            ROS_INFO_STREAM("State set to Home");
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
            ROS_INFO_STREAM("State set to " << state);
            return true;
        } else {
            ROS_INFO_STREAM("State set to " << robot_state.data);
        }

        ros::spinOnce();
    }
    return false;
}

void DVRKArm::moveJointRelative(int joint_idx, double movement)
{
   	// Collect data
    std::vector<double> currJoint = getJointStateCurrent();
    sensor_msgs::JointState new_position_joint = position_joint;
    new_position_joint.position[joint_idx] += movement;
    
    // Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    std::vector<double> distance(arm_typ.dof);
    for (int i = 0; i < arm_typ.dof; i++) {
    	distance[i] = abs(new_position_joint.position[i]-currJoint[i]);
    }
    for (int i = 0; i < arm_typ.dof; i++)
    {
    	if (distance[i] > arm_typ.maxDistJoint[i])
   		{
    		std::stringstream errstring;
    		errstring << "Desired joint vector too far from current." 
    					<< std::endl
    					<< "Desired joint vector:\t" 
    					<< new_position_joint.position
    					<< std::endl
    					<< "Current joint vector:\t" 
    					<< currJoint
    					<< std::endl
    					<< "Distance:\t" 
    					<< distance
    					<< std::endl
    					<< "MaxDistance:\t" 
    					<< arm_typ.maxDistJoint
    					<< std::endl;
			throw std::runtime_error(errstring.str());
		}
	}
    // End safety
    
    // Publish movement
    position_joint_pub.publish(new_position_joint);
    ros::spinOnce();
}

void DVRKArm::moveJointAbsolute(int joint_idx, double pos)
{
	// Collect data
    std::vector<double> currJoint = getJointStateCurrent();
    sensor_msgs::JointState new_position_joint = position_joint;
    new_position_joint.position[joint_idx] = pos;
    
    // Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    std::vector<double> distance(arm_typ.dof);
    for (int i = 0; i < arm_typ.dof; i++) {
    	distance[i] = abs(new_position_joint.position[i]-currJoint[i]);
    }
    for (int i = 0; i < arm_typ.dof; i++)
    {
    	if (distance[i] > arm_typ.maxDistJoint[i])
   		{
    		std::stringstream errstring;
    		errstring << "Desired joint vector too far from current." 
    					<< std::endl
    					<< "Desired joint vector:\t" 
    					<< new_position_joint.position
    					<< std::endl
    					<< "Current joint vector:\t" 
    					<< currJoint
    					<< std::endl
    					<< "Distance:\t" 
    					<< distance
    					<< std::endl
    					<< "MaxDistance:\t" 
    					<< arm_typ.maxDistJoint
    					<< std::endl;
			throw std::runtime_error(errstring.str());
		}
	}
    // End safety
    
    // Publish movement
    position_joint_pub.publish(new_position_joint);
    ros::spinOnce();
}

void DVRKArm::moveCartesianRelative(Eigen::Vector3d movement)
{
    // Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose += movement;
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
  	// Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    Pose::Distance d = currPose.dist(pose);
    if (d.cartesian > arm_typ.maxDistPose.cartesian ||	
    	d.angle > arm_typ.maxDistPose.angle ||  
    	d.jaw > arm_typ.maxDistPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Distance:\t" << d << std::endl
    			<< "MaxDistance:\t" << arm_typ.maxDistPose << std::endl;
		throw std::runtime_error(errstring.str());
	}
    
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void DVRKArm::moveCartesianAbsolute(Eigen::Vector3d position)
{
    // Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.position = position;
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
    // Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    Pose::Distance d = currPose.dist(pose);
    if (d.cartesian > arm_typ.maxDistPose.cartesian ||	
    	d.angle > arm_typ.maxDistPose.angle ||  
    	d.jaw > arm_typ.maxDistPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Distance:\t" << d << std::endl
    			<< "MaxDistance:\t" << arm_typ.maxDistPose << std::endl;
		throw std::runtime_error(errstring.str());
	}
    
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void DVRKArm::moveCartesianAbsolute(Eigen::Quaternion<double> orientation)
{
	// Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.orientation = orientation;
   	geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
   	
    // Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    Pose::Distance d = currPose.dist(pose);
    if (d.cartesian > arm_typ.maxDistPose.cartesian ||	
    	d.angle > arm_typ.maxDistPose.angle ||  
    	d.jaw > arm_typ.maxDistPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Distance:\t" << d << std::endl
    			<< "MaxDistance:\t" << arm_typ.maxDistPose << std::endl;
		throw std::runtime_error(errstring.str());
	}
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
}

void DVRKArm::moveCartesianAbsolute(Pose pose)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
    std_msgs::Float32 new_position_jaw = pose.toRosJaw();
    
    // Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    Pose::Distance d = currPose.dist(pose);
    if (d.cartesian > arm_typ.maxDistPose.cartesian ||	
    	d.angle > arm_typ.maxDistPose.angle ||  
    	d.jaw > arm_typ.maxDistPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Distance:\t" << d << std::endl
    			<< "MaxDistance:\t" << arm_typ.maxDistPose << std::endl;
		throw std::runtime_error(errstring.str());
	}
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
}

void DVRKArm::moveJawRelative(double movement)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.jaw += movement;
    std_msgs::Float32 new_position_jaw = pose.toRosJaw();
    
    // Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    Pose::Distance d = currPose.dist(pose.jaw);
    if (d.jaw > arm_typ.maxDistPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Distance:\t" << d << std::endl
    			<< "MaxDistance:\t" << arm_typ.maxDistPose << std::endl;
		throw std::runtime_error(errstring.str());
	}
    // End safety
    
    // Publish movement
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
}

void DVRKArm::moveJawAbsolute(double jaw)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.jaw = jaw;
    std_msgs::Float32 new_position_jaw = pose.toRosJaw();
    
    // Safety
    if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
    
    Pose::Distance d = currPose.dist(pose.jaw);
    if (d.jaw > arm_typ.maxDistPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Distance:\t" << d << std::endl
    			<< "MaxDistance:\t" << arm_typ.maxDistPose << std::endl;
		throw std::runtime_error(errstring.str());
	}
    // End safety
    
    // Publish movement
    position_jaw_pub.publish(new_position_jaw);
    ros::spinOnce();
}

/*
 * Trajectories
 */
void DVRKArm::playTrajectory(Trajectory<Eigen::Vector3d>& tr)
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

void DVRKArm::playTrajectory(Trajectory<Eigen::Quaternion<double>>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(tr[i]);
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

void DVRKArm::recordTrajectory(Trajectory<Eigen::Vector3d>& tr) 
{
	ros::Rate loop_rate(1.0/tr.dt);
	// Skip invalid points
	while (ros::ok() && getPositionCartesianCurrent().norm() < 0.001)
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
	while (ros::ok() && getPositionCartesianCurrent().norm() < 0.001)
  	{
  		loop_rate.sleep();
  	}
	while(ros::ok())
	{
		tr.addPoint(getPoseCurrent());
		loop_rate.sleep();
	}
}






