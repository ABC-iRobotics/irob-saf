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
			AutosurgAgent(nh, arm_names), grasp_vision(nh)
{	
	ctrl_client = 
		nh.serviceClient<irob_msgs::GetControlVariables>(
			"behavior/retract_crtl_srv");
}

Retractor::~Retractor()
{
	// TODO Auto-generated destructor stub
}


void Retractor::graspObject()
{
	ROS_INFO_STREAM("Creating service request...");
	irob_msgs::GetControlVariables srv;
  	srv.request.input.push_back(1.0);
  	srv.request.input.push_back(2.0);
  	
  	ROS_INFO_STREAM("Calling service...");
  	if (ctrl_client.call(srv))
  		ROS_INFO_STREAM("Service respond received: " << srv.response);
  	else
		ROS_INFO_STREAM("Service not responding");
		
		
	Eigen::Vector3d p = makeNaN<Eigen::Vector3d>();
	while (isnan(p) 
			&& ros::ok())
	{
		p = grasp_vision.getResult();
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO_STREAM("Object pose received: " << p);
	
	ROS_INFO_STREAM("Start grasp maneuver...");
	
	Eigen::Quaternion<double> grasp_ori 
				= BaseOrientations<CoordinateFrame::ROBOT,
					Eigen::Quaternion<double>>::DOWN_SIDEWAYS;
	
	Pose grasp_pose(p, grasp_ori, 0.0);
	
	double approach_dist = 20.0;
	Pose approach_pose = grasp_pose - (approach_dist *
			BaseDirections<CoordinateFrame::ROBOT,
			Eigen::Vector3d>::FORWARD);
	
	arms[0] -> grasp(grasp_pose, approach_pose, 3.0, 0.2, 20.0, 20.0);
	
	//Pose old_p = p;
	
	while(!arms[0] -> isGestureDone() && ros::ok())
	{
		
		/*p = vision.getResult();
		if ((p.position - old_p.position).norm() > 10.0)
		{
			ROS_INFO_STREAM("Initiating grasp preemt...");
			arms[0] -> grasp(p, approach_pose, 5.0, 0.5, 40.0, 40.0);
			old_p = p;
		}*/
		//ROS_INFO_STREAM(arms[0]->getFeedback().info);
		ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM(arms[0]->getResult().info);	
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



























