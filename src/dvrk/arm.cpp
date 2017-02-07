/*
 *  dvrk_arm.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *  
 */

#include "dvrk/arm.hpp"
#include <numeric>
#include <chrono>

namespace dvrk {

const std::string Arm::HOME_CMD
                    = "Home";
const std::string Arm::HOME_DONE
                    = "DVRK_READY";
const std::string Arm::STATE_POSITION_JOINT
                    = "DVRK_POSITION_JOINT";
                   // = "DVRK_POSITION_GOAL_JOINT";
const std::string Arm::STATE_POSITION_CARTESIAN
                    ="DVRK_POSITION_CARTESIAN";
                   // ="DVRK_POSITION_GOAL_CARTESIAN";


Arm::Arm(ros::NodeHandle nh, ArmTypes arm_typ, bool isActive): 
							nh(nh), arm_typ(arm_typ)
{

	// Subscribe and advertise topics
	subscribe(Topics::GET_ROBOT_STATE);
    subscribe(Topics::GET_STATE_JOINT_CURRENT);
    subscribe(Topics::GET_POSITION_CARTESIAN_CURRENT);
    subscribe(Topics::GET_ERROR);
    subscribe(Topics::GET_WARNING);
    
    if (isActive == ACTIVE)
    {
    	advertise(Topics::SET_ROBOT_STATE);
    	advertise(Topics::SET_POSITION_JOINT);
    	advertise(Topics::SET_POSITION_CARTESIAN);
    }
    
}

Arm::~Arm()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void Arm::robotStateCB(const std_msgs::String msg)
{
    robot_state = msg;
}

void Arm::stateJointCurrentCB(const sensor_msgs::JointStateConstPtr& msg) 
{
    position_joint = *msg;
}

void Arm::positionCartesianCurrentCB(
				const geometry_msgs::PoseStampedConstPtr& msg) 
{
     position_cartesian_current = *msg;
}

void Arm::errorCB(const std_msgs::String msg) 
{
     error = msg;
}

void Arm::warningCB(const std_msgs::String msg) 
{
     warning = msg;
}

bool Arm::subscribe(Topics topic) 
{
    if(topic == Topics::GET_ROBOT_STATE)
    {
        robot_state_sub = nh.subscribe<std_msgs::String>(
                        topic.getFullName(arm_typ), 1000,
                        &Arm::robotStateCB,this);
    }
    else if( topic == Topics::GET_STATE_JOINT_CURRENT)
    {
        state_joint_current_sub 
        				= nh.subscribe<sensor_msgs::JointState>(
                        topic.getFullName(arm_typ), 1000,
                        &Arm::stateJointCurrentCB,this);
    }
    else if( topic == Topics::GET_POSITION_CARTESIAN_CURRENT)
    {
        position_cartesian_current_sub 
        				= nh.subscribe<geometry_msgs::PoseStamped>(
                        topic.getFullName(arm_typ), 1000,
                        &Arm::positionCartesianCurrentCB,this);
    }
    else if( topic == Topics::GET_ERROR)
    {
       	error_sub 	= nh.subscribe<std_msgs::String>(
                        topic.getFullName(arm_typ), 1000,
                        &Arm::errorCB,this);
    }
    else if( topic == Topics::GET_WARNING)
    {
       	warning_sub 	= nh.subscribe<std_msgs::String>(
                        topic.getFullName(arm_typ), 1000,
                        &Arm::warningCB,this);
    }
    else
    {
        ROS_WARN_STREAM("Subscribing to invalid topic " 
        	<<  topic.getFullName(arm_typ));
        return false;
    }

    ROS_DEBUG_STREAM("Subscribed to topic " 
    	<< topic.getFullName(arm_typ));
    return true;
}

bool Arm::advertise(Topics topic) 
{
    if(topic == Topics::SET_ROBOT_STATE)
    {
        robot_state_pub = nh.advertise<std_msgs::String>(
                                    topic.getFullName(arm_typ), 1000);
    }
    else if(topic == Topics::SET_POSITION_JOINT)
    {
        position_joint_pub = nh.advertise<sensor_msgs::JointState>(
                                   topic.getFullName(arm_typ), 1000);
    }
    else if(topic == Topics::SET_POSITION_CARTESIAN)
    {
        position_cartesian_pub = nh.advertise<geometry_msgs::Pose>(
                                   topic.getFullName(arm_typ), 1000);
    }
    else 
    {
         ROS_WARN_STREAM("Advertising invalid topic " << topic.getFullName(arm_typ));
         return false;
    }
    ROS_DEBUG_STREAM("Advertised topic " << topic.getFullName(arm_typ));
    return true;
}

/*
 * DVRK actions
 */
double Arm::getJointStateCurrent(int index)
{
	ros::spinOnce();
	if (index > ((int)position_joint.position.size())-1)
	{
		ROS_WARN_STREAM("Joint index too big or no joint position information.");
		return NAN;
	}
 	return position_joint.position[index];
}

std::vector<double> Arm::getJointStateCurrent()
{
	ros::spinOnce();
	std::vector<double> ret(arm_typ.dof);
	
		ret = position_joint.position;
 	return ret;
}
 
Eigen::Vector3d Arm::getPositionCartesianCurrent()
{
 	ros::spinOnce();
 	Eigen::Vector3d ret(position_cartesian_current.pose.position.x,
 						position_cartesian_current.pose.position.y,
 						position_cartesian_current.pose.position.z);
    return ret;
}

Eigen::Quaternion<double> Arm::getOrientationCartesianCurrent()
{
	ros::spinOnce();
 	Eigen::Quaternion<double> ret(position_cartesian_current.pose.orientation.x,
 		position_cartesian_current.pose.orientation.y, 
 		position_cartesian_current.pose.orientation.z, 
 		position_cartesian_current.pose.orientation.w);
    return ret;
}

Pose Arm::getPoseCurrent()
{
 	ros::spinOnce();
 	Pose ret(position_cartesian_current, 0.0);
 	return ret;

}


bool Arm::home()
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

bool Arm::setRobotState(std::string state)
{
    while (true) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << state;
        msg.data = ss.str();
        robot_state_pub.publish(msg);
        ros::Duration(0.5).sleep();
		ros::spinOnce();
		checkErrors();
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

void Arm::moveJointRelative(int joint_idx, double movement, double dt)
{
   	// Collect data
    std::vector<double> currJoint = getJointStateCurrent();
    sensor_msgs::JointState new_position_joint = position_joint;
    new_position_joint.position[joint_idx] += movement;
    
    // Safety
    checkErrors();
    checkNaNJoint(new_position_joint);
    checkVelJoint(new_position_joint, currJoint, dt);
    // End safety
    
    // Publish movement
    position_joint_pub.publish(new_position_joint);
    ros::spinOnce();
}

void Arm::moveJointAbsolute(int joint_idx, double pos, double dt)
{
	// Collect data
    std::vector<double> currJoint = getJointStateCurrent();
    sensor_msgs::JointState new_position_joint = position_joint;
    new_position_joint.position[joint_idx] = pos;
    
   	// Safety
    checkErrors();
    checkNaNJoint(new_position_joint);
    checkVelJoint(new_position_joint, currJoint, dt);
    // End safety
    
    // Publish movement
    position_joint_pub.publish(new_position_joint);
    ros::spinOnce();
}

void Arm::moveCartesianRelative(Eigen::Vector3d movement, double dt)
{
    // Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose += movement;
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
  	
  	// Safety
    checkErrors();
    checkNaNCartesian(pose);
    checkVelCartesian(pose, currPose, dt);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void Arm::moveCartesianAbsolute(Eigen::Vector3d position, double dt)
{
    // Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.position = position;
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
    
    // Safety
    checkErrors();
    checkNaNCartesian(pose);
    checkVelCartesian(pose, currPose, dt);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
    //ros::Duration(0.1).sleep();

}

void Arm::moveCartesianAbsolute(Eigen::Quaternion<double> orientation, double dt)
{
	// Collect data
	Pose currPose = getPoseCurrent();
    Pose pose = currPose;
    pose.orientation = orientation;
   	geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
   	
    // Safety
    checkErrors();
    checkNaNCartesian(pose);
    checkVelCartesian(pose, currPose, dt);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
}

void Arm::moveCartesianAbsolute(Pose pose, double dt)
{
	// Collect data
    Pose currPose = getPoseCurrent();
    geometry_msgs::Pose new_position_cartesian = pose.toRosPose();
    
    // Safety
    checkErrors();
    checkNaNCartesian(pose);
    checkVelCartesian(pose, currPose, dt);
    // End safety
    
    // Publish movement
    position_cartesian_pub.publish(new_position_cartesian);
    ros::spinOnce();
}

void Arm::checkErrors()
{
	if (!warning.data.empty())
    {
    	ROS_WARN_STREAM(warning.data);
    	warning.data.clear();
    }
    
    if (!error.data.empty())
   		throw std::runtime_error(error.data);
}

void Arm::checkVelCartesian(const Pose& pose, 
								const Pose& currPose, double dt)
{
	Pose::Distance d = currPose.dist(pose)/dt;
    if (d.cartesian > arm_typ.maxVelPose.cartesian ||	
    	d.angle > arm_typ.maxVelPose.angle ||  
    	d.jaw > arm_typ.maxVelPose.jaw)
    {
    	std::stringstream errstring;
    	errstring << "Desired pose too far from current."<< std::endl
    			<< "Desired pose:\t" << pose << std::endl
    			<< "Current pose:\t" << currPose << std::endl
    			<< "Velocity:\t" << d << std::endl
    			<< "MaxVelocity:\t" << arm_typ.maxVelPose << std::endl;
    	//ROS_ERROR_STREAM(errstring.str());
		throw std::runtime_error(errstring.str());
	}
}

void Arm::checkNaNCartesian(const Pose& pose)
{
    if (pose.isNaN())
    {
    	std::stringstream errstring;
    	errstring << "Desired pose is NaN:\t"<< std::endl
    			<< pose << std::endl;
    	//ROS_ERROR_STREAM(errstring.str());
		throw std::runtime_error(errstring.str());
	}
}

void Arm::checkVelJoint(const sensor_msgs::JointState& new_position_joint, 
						const std::vector<double>& currJoint, double dt)
{
	std::vector<double> distance(arm_typ.dof);
    for (int i = 0; i < arm_typ.dof; i++) {
    	distance[i] = std::abs(new_position_joint.position[i]-currJoint[i])/dt;
    }
    for (int i = 0; i < arm_typ.dof; i++)
    { 		
    	if (distance[i] > arm_typ.maxVelJoint[i])
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
    					<< "Velocity:\t" 
    					<< distance
    					<< std::endl
    					<< "MaxVelocity:\t" 
    					<< arm_typ.maxVelJoint
    					<< std::endl;
    		//ROS_ERROR_STREAM(errstring.str());
			throw std::runtime_error(errstring.str());
		}
	}
}

void Arm::checkNaNJoint(const sensor_msgs::JointState& new_position_joint)
{
	bool foundNaN = false;
    for (int i = 0; i < arm_typ.dof; i++) {
    	foundNaN = foundNaN || std::isnan(new_position_joint.position[i]);
    }		
    if (foundNaN)
   	{
    	std::stringstream errstring;
    	errstring << "Desired joint vector is NaN:\t" 
    					<< new_position_joint.position
    					<< std::endl;
    		//ROS_ERROR_STREAM(errstring.str());
			throw std::runtime_error(errstring.str());
		}
}

/*
 * Trajectories
 */
void Arm::playTrajectory(Trajectory<Eigen::Vector3d>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	//int cnt = 0;
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		//auto start = std::chrono::high_resolution_clock::now();
		
		moveCartesianAbsolute(tr[i],tr.dt);
		
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

void Arm::playTrajectory(Trajectory<Eigen::Quaternion<double>>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(tr[i],tr.dt);
		loop_rate.sleep();
	}
}

void Arm::playTrajectory(Trajectory<Pose>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveCartesianAbsolute(tr[i],tr.dt);
		loop_rate.sleep();
	}
}

void Arm::playTrajectory(int jointIndex, Trajectory<double>& tr)
{
	ros::Rate loop_rate(1.0/tr.dt);
	for (int i = 0; i < tr.size() && ros::ok(); i++)
	{
		moveJointAbsolute(jointIndex, tr[i],tr.dt);
		loop_rate.sleep();
	}
}

void Arm::recordTrajectory(Trajectory<Eigen::Vector3d>& tr) 
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

void Arm::recordTrajectory(Trajectory<Pose>& tr) 
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


}



