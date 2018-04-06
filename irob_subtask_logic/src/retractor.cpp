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
			AutosurgAgent(nh, arm_names), grasp_vision(nh, "target"),
											 manipulate_vision(nh,"angles")
{	
	ctrl_client = 
		nh.serviceClient<irob_msgs::GetControlVariables>(
			"behavior/retract_ctrl_srv");
}

Retractor::~Retractor()
{
	// TODO Auto-generated destructor stub
}


void Retractor::graspObject()
{
	/*	
	Eigen::Vector3d p = makeNaN<Eigen::Vector3d>();
	while (isnan(p) 
			&& ros::ok())
	{
		p = grasp_vision.getResult();
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO_STREAM("Object pose received: " << p);
	
	ROS_INFO_STREAM("Start grasp maneuver...");
	
	//Eigen::Quaternion<double> grasp_ori 
	//			= BaseOrientations<CoordinateFrame::ROBOT,
	//				Eigen::Quaternion<double>>::DOWN_SIDEWAYS;
	
	Eigen::Quaternion<double> 
			grasp_ori(0.0123977130067,-0.16942504864,-0.163500064346,0.971807159655);


	Pose grasp_pose(p, grasp_ori, 0.0);
	
	double approach_dist = 30.0;
	Eigen::Vector3d z(0.0, 0.0, 1.0);
	Pose approach_pose = grasp_pose - ((grasp_ori.toRotationMatrix() * z) * approach_dist);
	//Pose approach_pose = grasp_pose - (approach_dist *(z * grasp_ori.toRotationMatrix()));
	ROS_INFO_STREAM("Grasp pose: " << grasp_pose);
	arms[0] -> grasp(grasp_pose, approach_pose, 8.0, 0.05, 20.0, 20.0);
	
	
  while(!arms[0] -> issurgemeDone() && ros::ok())
	{
		
		ros::Duration(0.1).sleep();
	}
	
	ROS_INFO_STREAM(arms[0]->getResult().info);	*/
	ROS_INFO_STREAM("Grasping succeeded");
	

		
	double diff = 100.0;
	double threshold = 2.0;
	
	while (diff >= threshold && ros::ok())
	{
		
		/*for (int i = 20; i > 0; i--)
		{
			ROS_INFO_STREAM(i);
			ros::Duration(1).sleep();
		}
		ROS_INFO_STREAM("START");*/
		std::vector<double> angles(manipulate_vision.getResult());
	
		irob_msgs::GetControlVariables srv;
  		srv.request.input.push_back(angles[0]);
  		srv.request.input.push_back(angles[1]);
  		srv.request.input.push_back(angles[2]);
  	
  		if (ctrl_client.call(srv))
  			ROS_INFO_STREAM("Service respond received: " << srv.response);
  		else
			ROS_INFO_STREAM("Service not responding");
		
		Eigen::Vector3d movement(0.0, srv.response.output[0], 	srv.response.output[1]);
		diff = movement.norm();
		
		arms[0] -> manipulate(movement);
		
    while(!arms[0] -> issurgemeDone() && ros::ok())
		{
		
			ros::Duration(0.1).sleep();
		}	
		//ROS_INFO_STREAM("DONE");

	}
	
	ROS_INFO_STREAM(arms[0]->getResult().info);	
	ROS_INFO_STREAM("Retraction succeeded");
	

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
	
	
    
    // Startsurgeme server
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



























