/*
 * 	retractor.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-13
 *  
 */

#include <irob_subtask/retractor.hpp>


namespace ias {


Retractor::Retractor(ros::NodeHandle nh, 
					std::vector<std::string> arm_names): 
			AutosurgAgent(nh, arm_names), vision(nh)
{	
	// 
}

Retractor::~Retractor()
{
	// TODO Auto-generated destructor stub
}


void Retractor::graspObject()
{
	Pose p = makeNaN<Pose>();
	while (isnan(p) 
			&& ros::ok())
	{
		p = vision.getResult();
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO_STREAM("Object pose received: " << p);
	
	ROS_INFO_STREAM("Start grasp maneuver...");
	double approach_dist = 10.0;
	Pose approach_pose = p - (approach_dist *
			AbstractDirections<CoordinateFrame::ROBOT,
			Eigen::Vector3d>::DOWN);
	
	arms[0] -> grasp(p, approach_pose, 5.0, 0.5, 40.0, 40.0);
	Pose old_p = p;
	while(!arms[0] -> isGestureDone() && ros::ok())
	{
		
		p = vision.getResult();
		if ((p.position - old_p.position).norm() > 10.0)
		{
			ROS_INFO_STREAM("Initiating grasp preemt...");
			arms[0] -> grasp(p, approach_pose, 5.0, 0.5, 40.0, 40.0);
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
    	Retractor retr(nh, arm_names);
    	
  	   	retr.graspObject();	    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}



























