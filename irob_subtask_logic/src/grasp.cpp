/*
 * 	grasp.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *
 */

#include <irob_subtask_logic/grasp.hpp>


namespace saf {


Grasp::Grasp(ros::NodeHandle nh, ros::NodeHandle priv_nh,
             std::vector<std::string> arm_names):
  AutosurgAgent(nh, priv_nh, arm_names), vision(nh, "target")
{	
  //
}

Grasp::~Grasp()
{
  // TODO Auto-generated destructor stub
}


void Grasp::graspObject()
{

  // NaN pose received, until the vision node starts
  Eigen::Affine3d p = makeNaN<Eigen::Affine3d>();
  while (isnan(p)
         && ros::ok())
  {
    p = vision.getResult();
    ros::Duration(0.1).sleep();
  }

  //ROS_INFO_STREAM("Object pose received: " << p);

  ROS_INFO_STREAM("Start grasp maneuver...");

  // Calculate approach position
  double approach_dist = 10.0;
  Eigen::Affine3d approach_pose(Eigen::Translation3d(approach_dist *
                            BaseDirections<CoordinateFrame::CAMERA,
                            Eigen::Vector3d>::BACKWARD).inverse() * p);

  // Send grasp surgeme action to the surgeme server.
  arms[0] -> grasp(p, approach_pose, 5.0, 0.95, 20.0, 20.0);
  Eigen::Affine3d old_p = p;
  // Wait for action to be finished
  while(!arms[0] -> isSurgemeDone() && ros::ok())
  {

    // Receive action result
    p = vision.getResult();

    // Preempt if object moves away
    if ((p.translation() - old_p.translation()).norm() > 10.0)
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



  // Start autonomous agent
  try {
    Grasp pnp(nh, priv_nh, arm_names);

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



























