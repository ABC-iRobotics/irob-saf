/*
 * 	peg_transfer.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-01-11
 *
 */

#include <irob_subtask_logic/peg_transfer.hpp>


namespace saf {


PegTransfer::PegTransfer(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                         std::vector<std::string> arm_names):
  AutosurgAgent(nh, priv_nh, arm_names), vision(nh, "target")
{
  loadBoardDescriptor(priv_nh);
}

PegTransfer::~PegTransfer()
{
  // TODO Auto-generated destructor stub
}

void PegTransfer::loadBoardDescriptor(ros::NodeHandle priv_nh)
{

  priv_nh.getParam("peg_h", peg_h);
  priv_nh.getParam("object_h", object_h);
  priv_nh.getParam("object_d", object_d);
  priv_nh.getParam("object_wall_d", object_wall_d);

  priv_nh.getParam("offs_x", offs_x);
  priv_nh.getParam("offs_y", offs_y);
  priv_nh.getParam("offs_z" , offs_z);


  std::vector<double> param_board_t;
  priv_nh.getParam("board_t", param_board_t);

  std::vector<double> param_peg_positions;
  priv_nh.getParam("peg_positions", param_peg_positions);

  for (int i = 0; i < board_t.rows(); i++)
    board_t(i) = param_board_t[i];

  double rows = param_peg_positions.size() / 3;
  for (int i = 0; i < rows; i++)
     peg_positions.push_back(Eigen::Vector3d(
                                param_peg_positions[i * 3],
                                param_peg_positions[(i * 3) + 1],
                                param_peg_positions[(i * 3) + 2]));

  ROS_INFO_STREAM(
        "Board descriptor read.");
        //: "<< std::endl << board_t << std::endl << peg_positions);
}

Eigen::Affine3d PegTransfer::poseToCameraFrame(const Eigen::Affine3d& pose,
                                      const Eigen::Affine3d& tr)
{
  Eigen::Affine3d ret(pose);
  ret = Eigen::Translation3d(board_t) * ret;
  ret = tr * ret;  // Ori OK
  return ret;
}

Eigen::Affine3d PegTransfer::poseToWorldFrame(const Eigen::Affine3d& pose,
                                   const Eigen::Affine3d& tr)
{
  Eigen::Affine3d ret(pose);
  ret = tr.inverse() * ret;
  ret = Eigen::Translation3d(board_t).inverse() * ret;
  return ret;

}


void PegTransfer::doPegTransfer()
{



  double compress_rate = 0.2;
  int peg_idx_on = 0;
  int peg_idx_to = 6;
  int increment = 1;

  irob_msgs::Environment e;
  // Read marker transform
  ROS_INFO_STREAM("Waiting for data from vision...");
  e = makeNaN<irob_msgs::Environment>();
  while (isnan(e) && ros::ok())
  {
    e = vision.getResult();
    ros::Duration(0.1).sleep();
  }

  //Vision data received
  Eigen::Quaternion<double> ori_world =
        BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
        DOWN_SIDEWAYS;

  Eigen::Quaternion<double> grasp_ori_world(ori_world);

  Eigen::Translation3d offset = Eigen::Translation3d(Eigen::Vector3d(offs_x, offs_y, offs_z));

  while(ros::ok())
  {

    // Grasp object
    ROS_INFO_STREAM("Grasping object on rod " << peg_idx_on << "...");

    Eigen::Affine3d grasp_pose_on(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                                    e.objects[peg_idx_on].grasp_pose));

    grasp_pose_on = offset * grasp_pose_on;


    Eigen::Affine3d grasp_approach_pose_on(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                             e.objects[peg_idx_on].approach_pose));

    grasp_approach_pose_on = offset * grasp_approach_pose_on;

    arms[0]->grasp(grasp_pose_on, grasp_approach_pose_on, object_wall_d, compress_rate, speed_cartesian, speed_jaw);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    ROS_INFO_STREAM("Grased object on rod " << peg_idx_on << "...");
    ros::Duration(100.0).sleep();

    // Place to new rod
    ROS_INFO_STREAM("Placing object to rod " << peg_idx_to << "...");

    /*Eigen::Affine3d place_approach_pose_on(
          grasp_ori_world * Eigen::Translation3d(peg_positions[peg_idx_on]));
    place_approach_pose_on =
        Eigen::Translation3d(approach_post_grasp_translate_world)
        * place_approach_pose_on;
    place_approach_pose_on = poseToCameraFrame(place_approach_pose_on, e);
    std::vector<Eigen::Affine3d> waypoints;
    waypoints.push_back(place_approach_pose_on);

    Eigen::Affine3d place_pose_to(
          grasp_ori_world * Eigen::Translation3d(peg_positions[peg_idx_to]));
    place_pose_to =
        Eigen::Translation3d(grasp_translate_world)
        * place_pose_to;
    place_pose_to = poseToCameraFrame(place_pose_to, e);

    Eigen::Affine3d place_approach_pose_to(
          grasp_ori_world * Eigen::Translation3d(peg_positions[peg_idx_to]));
    place_approach_pose_to =
        Eigen::Translation3d(approach_post_grasp_translate_world)
        * place_approach_pose_to;
    place_approach_pose_to = poseToCameraFrame(place_approach_pose_to, e);

    arms[0] -> place(place_pose_to, place_approach_pose_to, speed_cartesian, waypoints);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }


    // Release
    Eigen::Affine3d release_approach_pose_to(
          grasp_ori_world * Eigen::Translation3d(peg_positions[peg_idx_to]));
    release_approach_pose_to =
        Eigen::Translation3d(approach_pre_grasp_translate_world)
        * release_approach_pose_to;
    release_approach_pose_to = poseToCameraFrame(release_approach_pose_to, e);

    arms[0] -> release(release_approach_pose_to, object_wall_d, speed_cartesian, speed_jaw);
    ROS_INFO_STREAM("Release object...");
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    ROS_INFO_STREAM("Peg-transfer subtask done");

    // Set new traget
    peg_idx_on+=increment;
    peg_idx_to+=increment;
    if (peg_idx_to == 12) {
      ROS_INFO_STREAM("All pegs transfered");
      peg_idx_on = 6;
      peg_idx_to = 0;

      ros::Duration(1.0).sleep();
    } else if (peg_idx_to == 6) {
      ROS_INFO_STREAM("All pegs transfered");
      peg_idx_on = 0;
      peg_idx_to = 6;

      ros::Duration(1.0).sleep();
    }*/


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
    PegTransfer pnp(nh ,  priv_nh, arm_names);

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







