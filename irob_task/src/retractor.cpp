/*
 * 	retractor.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-09-05
 *  
 */

#include <irob_task/retractor.hpp>


namespace ias {


Retractor::Retractor(ros::NodeHandle nh, 
					std::vector<std::string> arm_names): 
			nh(nh), arm_names(arm_names), maneuver(nh, arm_names), vision(nh)
{

	// Subscribe and advertise topics
	
	
}

Retractor::~Retractor()
{
	// TODO Auto-generated destructor stub
}


void Retractor::graspTissue()
{
	Eigen::Vector3d p = makeNaN<Eigen::Vector3d>();
	Eigen::Vector3d old_p;
	while (isnan(p) 
			&& ros::ok())
	{
		p = vision.getResult();
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO_STREAM("Object position received: " << p);
	
	Pose pose = maneuver.getPoseCurrent(arm_names[0]);
	pose.position = p;
	
	ROS_INFO_STREAM("Start grasp maneuver...");
	maneuver.grasp(arm_names[0], pose, 30, 0, 10.0);
	old_p = p;
	while(!maneuver.isManeuverDone() && ros::ok())
	{
		
		p = vision.getResult();
		if ((p - old_p).norm() > 10.0)
		{
			ROS_INFO_STREAM("Initiating grasp preemt...");
			maneuver.grasp(arm_names[0], pose, 30, 0, 10.0);
			old_p = p;
		}
		ros::Duration(0.1).sleep();
	}
		
	ROS_INFO_STREAM("Grasping succeeded");
}

void Retractor::retract()
{
	Eigen::Vector3d p = makeNaN<Eigen::Vector3d>();
	Eigen::Vector3d old_p;
	while (isnan(p) 
			&& ros::ok())
	{
		p = vision.getResult();
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO_STREAM("Object position received: " << p);
	
	Pose pose = maneuver.getPoseCurrent(arm_names[0]);
	pose.position = p;
	
	ROS_INFO_STREAM("Start grasp maneuver...");
	maneuver.grasp(arm_names[0], pose, 30, 0, 10.0);
	old_p = p;
	while(!maneuver.isManeuverDone() && ros::ok())
	{
		
		p = vision.getResult();
		if ((p - old_p).norm() > 10.0)
		{
			ROS_INFO_STREAM("Initiating grasp preemt...");
			maneuver.grasp(arm_names[0], pose, 30, 0, 10.0);
			old_p = p;
		}
		ros::Duration(0.1).sleep();
	}
		
	ROS_INFO_STREAM("Grasping succeeded");
}

}

using namespace ias;

/**
 * Maneuver server main 
 */
int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "test_task");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

	std::vector<std::string> arm_names;
	priv_nh.getParam("arm_names", arm_names);
	
	
    
    // StartGesture server
  	try {
    	Retractor r(nh, arm_names);
    	
  	   	r.graspTissue();
  	   	r.retract();	    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}



























