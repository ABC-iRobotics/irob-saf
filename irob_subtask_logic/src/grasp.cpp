/*
 * 	grasp.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *  
 */

#include <irob_subtask/grasp.hpp>


namespace saf {


Grasp::Grasp(ros::NodeHandle nh, 
					std::vector<std::string> arm_names): 
			AutosurgAgent(nh, arm_names), vision(nh, "target")
{	
	// 
}

Grasp::~Grasp()
{
	// TODO Auto-generated destructor stub
}


void Grasp::graspObject()
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
			BaseDirections<CoordinateFrame::CAMERA,
      Eigen::Vector3d>::BACKWARD);

  arms[0] -> grasp(p, approach_pose, 5.0, 0.95, 20.0, 20.0);
	Pose old_p = p;
  while(!arms[0] -> isSurgemeDone() && ros::ok())
	{
		
		p = vision.getResult();
		if ((p.position - old_p.position).norm() > 10.0)
		{
			ROS_INFO_STREAM("Initiating grasp preemt...");
      arms[0] -> grasp(p, approach_pose, 5.0, 0.95, 20.0, 20.0);
			old_p = p;
		}
		ros::Duration(0.1).sleep();
	}
		
	ROS_INFO_STREAM("Grasping succeeded");
}

}

using namespace saf;

/**
 * Maneuver server main 
 */
int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "dummy_grasp");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

	std::vector<std::string> arm_names;
	priv_nh.getParam("arm_names", arm_names);
	
	
    
    // Startsurgeme server
  	try {
    	Grasp pnp(nh, arm_names);
    	
  	   	pnp.graspObject();	    	
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}



























