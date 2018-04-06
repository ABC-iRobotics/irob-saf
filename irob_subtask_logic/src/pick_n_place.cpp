/*
 * 	pick_n_place.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-01-11
 *
 */

#include <irob_subtask/pick_n_place.hpp>


namespace ias {


PicknPlace::PicknPlace(ros::NodeHandle nh,
                       std::vector<std::string> arm_names, Eigen::Quaternion<double> ori, Eigen::Vector3d dp):
  AutosurgAgent(nh, arm_names), vision(nh, "target"), ori(ori), dp(dp)
{
  //
}

PicknPlace::~PicknPlace()
{
  // TODO Auto-generated destructor stub
}


void PicknPlace::doPnP()
{

  Pose dist_pose(dp, ori, 0.0);
  double compress_rate = 0.705;

  ROS_INFO_STREAM("Starting Pick-and-Place behaviour...");

  int tube_idx_on = 6;
  int tube_idx_to = 7;
  int increment = 1;

  // Go to distant position
  ROS_INFO_STREAM("Go to distant position...");
  arms[0] -> nav_to_pos(dist_pose);
  while(!arms[0] -> issurgemeDone() && ros::ok())
  {
    ros::Duration(0.1).sleep();
  }

  // Read environment data
  ROS_INFO_STREAM("Waiting for data from vision...");
  irob_msgs::Environment e = makeNaN<irob_msgs::Environment>();
  while (e.valid != irob_msgs::Environment::VALID
         && ros::ok())
  {
    e = vision.getResult();
    ros::Duration(0.1).sleep();
  }




  while(ros::ok())
  {

    // Go to distant position
    ROS_INFO_STREAM("Go to distant position...");
    arms[0] -> nav_to_pos(dist_pose);
    while(!arms[0] -> issurgemeDone() && ros::ok())
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


    // Grasp object
    ROS_INFO_STREAM("Grasping object on rod " << tube_idx_on << "...");
    Pose grasp_pose_on(e.objects[tube_idx_on].grasp_position, ori, 0.0);
    Pose approach_pose_on(e.objects[tube_idx_on].approach_position, ori, 0.0);
    double d = e.objects[tube_idx_on].grasp_diameter;
    arms[0]->grasp(grasp_pose_on, approach_pose_on, d, compress_rate);
    while(!arms[0] -> issurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Place to new rod
    ROS_INFO_STREAM("Placing object to rod " << tube_idx_to << "...");
    Pose grasp_pose_to(e.objects[tube_idx_to].grasp_position, ori, 0.0);
    Pose approach_pose_to(e.objects[tube_idx_to].approach_position, ori, 0.0);
    std::vector<Pose> waypoints;
    waypoints.push_back(approach_pose_on);
    //waypoints.push_back(dist_pose);
    arms[0] -> place(grasp_pose_to, approach_pose_to, d, waypoints);
    while(!arms[0] -> issurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }


    // Release
    arms[0] -> release(approach_pose_to, d);
    ROS_INFO_STREAM("Release object...");
    while(!arms[0] -> issurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    ROS_INFO_STREAM("Pick-and-place task done");

    // Set new traget
    tube_idx_on = tube_idx_to;
    tube_idx_to+=increment;
    if (tube_idx_to == 12) {
      tube_idx_on = 6;
      tube_idx_to  = 7;
    }

  }

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

  std::vector<double> ori;
  priv_nh.getParam("ori", ori);

  Eigen::Quaternion<double> quat_ori(ori[0], ori[1], ori[2], ori[3]);

  std::vector<double> dp;
  priv_nh.getParam("dp", dp);

  Eigen::Vector3d vec_dp(dp[0], dp[1], dp[2]);

  // Startsurgeme server
  try {
    PicknPlace pnp(nh, arm_names, quat_ori, vec_dp);

    pnp.doPnP();

    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}







