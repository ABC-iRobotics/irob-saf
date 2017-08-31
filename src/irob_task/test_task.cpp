/*
 * 	test_task.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-08-30
 *  
 */

#include "irob_task/test_task.hpp"


namespace ias {


TestTask::TestTask(ros::NodeHandle nh, 
					std::vector<std::string> arm_names): 
			nh(nh), arm_names(arm_names), maneuver(nh, arm_names), vision(nh)
{

	// Subscribe and advertise topics
	
	
}

TestTask::~TestTask()
{
	// TODO Auto-generated destructor stub
}


void TestTask::graspObject()
{
	Eigen::Vector3d p(std::numeric_limits<double>::quiet_NaN(),
						std::numeric_limits<double>::quiet_NaN(),
						std::numeric_limits<double>::quiet_NaN());
	while ((std::isnan(p.x()) || std::isnan(p.y()) || std::isnan(p.z())) 
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
	
	while(!maneuver.isGraspDone() && ros::ok())
	{
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
    	TestTask tt(nh, arm_names);
    	
  	   	tt.graspObject();	    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}



























