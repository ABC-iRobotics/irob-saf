/*
 * 	peg_transfer.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-01-11
 *
 */

#include <irob_subtask_logic/peg_transfer.hpp>


namespace saf {


PegTransfer::PegTransfer(ros::NodeHandle nh,
                         std::vector<std::string> arm_names):
  AutosurgAgent(nh, arm_names), vision(nh, "target")
{
  //
}

PegTransfer::~PegTransfer()
{
  // TODO Auto-generated destructor stub
}


void PegTransfer::doPegTransfer()
{
  irob_msgs::Environment e;
  // Read environment data
  ROS_INFO_STREAM("Waiting for data from vision...");
  e = makeNaN<irob_msgs::Environment>();
  while (e.valid != irob_msgs::Environment::VALID
         && ros::ok())
  {
    e = vision.getResult();
    ros::Duration(0.1).sleep();
  }

  //Environment data received

  Eigen::Quaternion<double> ori_phantom =
        BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
        DOWN_SIDEWAYS;

  Pose dp_phantom(Eigen::Vector3d(90.0, 0.0, 70.0), ori_phantom, 0.0);

  Pose dist_pose(dp_phantom.transform(e.tf_phantom));

  Eigen::Quaternion<double> disp_ori(dist_pose.orientation);
  Eigen::Quaternion<double> grasp_ori(disp_ori);

  Pose grasp_translate_phantom(
        Eigen::Vector3d(8.0, 0.0, 0.0), ori_phantom, 0.0);

  Eigen::Vector3d grasp_translate(
        grasp_translate_phantom.transform(e.tf_phantom).position
        - Pose().transform(e.tf_phantom).position);
          // Remove translation

  double compress_rate = 0.2;
  int tube_idx_on = 0;
  int tube_idx_to = 6;
  int increment = 1;
  double speed_cartesian = 30.0;
  double speed_jaw = 30.0;


  while(ros::ok())
  {

   /* // Go to distant position
    ROS_INFO_STREAM("Go to distant position...");
    arms[0] -> nav_to_pos(dist_pose, speed_cartesian);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Wait to let the phantom be rearranged
    ros::Duration(1.0).sleep();


    // Read environment data
    ROS_INFO_STREAM("Waiting for data from vision...");
    irob_msgs::Environment e = makeNaN<irob_msgs::Environment>();
    while (e.valid != irob_msgs::Environment::VALID
           && ros::ok())
    {
      e = vision.getResult();
      ros::Duration(0.1).sleep();
    }
    dist_pose = dp_phantom.transform(e.tf_phantom);

    disp_ori = dist_pose.orientation;
    grasp_ori = disp_ori;


    grasp_translate =
          grasp_translate_phantom.transform(e.tf_phantom).position
          - Pose().transform(e.tf_phantom).position;
            // Remove translation
    */

    // Grasp object
    ROS_INFO_STREAM("Grasping object on rod " << tube_idx_on << "...");
    Pose grasp_pose_on(e.objects[tube_idx_on].grasp_position, grasp_ori, 0.0);
     grasp_pose_on += grasp_translate;
    Pose approach_pose_on(e.objects[tube_idx_on].approach_position, grasp_ori, 0.0);
    approach_pose_on += grasp_translate;
    double d = e.objects[tube_idx_on].grasp_diameter;
    arms[0]->grasp(grasp_pose_on, approach_pose_on, d, compress_rate, speed_cartesian, speed_jaw);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Place to new rod
    ROS_INFO_STREAM("Placing object to rod " << tube_idx_to << "...");
    Pose grasp_pose_to(e.objects[tube_idx_to].grasp_position, grasp_ori, 0.0);
    grasp_pose_to += grasp_translate;
    Pose approach_pose_to(e.objects[tube_idx_to].approach_position, grasp_ori, 0.0);
    approach_pose_to += grasp_translate;
    std::vector<Pose> waypoints;
    waypoints.push_back(approach_pose_on);
    //waypoints.push_back(dist_pose);
    arms[0] -> place(grasp_pose_to, approach_pose_to, speed_cartesian, waypoints);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }


    // Release
    arms[0] -> release(approach_pose_to, d, speed_cartesian, speed_jaw);
    ROS_INFO_STREAM("Release object...");
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    ROS_INFO_STREAM("Peg-transfer subtask done");

    // Set new traget
    tube_idx_on+=increment;
    tube_idx_to+=increment;
    if (tube_idx_to == 12) {
      ROS_INFO_STREAM("All pegs transfered");
      tube_idx_on = 6;
      tube_idx_to = 0;

      ros::Duration(1.0).sleep();
    }

  }

}
}

using namespace saf;

/**
 * Maneuver server main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
  ros::init(argc, argv, "peg_transfer");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::vector<std::string> arm_names;
  priv_nh.getParam("arm_names", arm_names);


  // Start autonomous agent
  try {
    PegTransfer pnp(nh, arm_names);

    pnp.doPegTransfer();

    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}







