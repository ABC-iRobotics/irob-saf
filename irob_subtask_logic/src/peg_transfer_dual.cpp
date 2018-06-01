/*
 * 	peg_transfer_dual.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-06-01
 *
 */

#include <irob_subtask_logic/peg_transfer_dual.hpp>


namespace saf {


PegTransferDual::PegTransferDual(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                                 std::vector<std::string> arm_names):
  AutosurgAgent(nh, priv_nh, arm_names), vision(nh, "target")
{
  loadBoardDescriptor(priv_nh);
}

PegTransferDual::~PegTransferDual()
{
  // TODO Auto-generated destructor stub
}

void PegTransferDual::loadBoardDescriptor(ros::NodeHandle priv_nh)
{

  priv_nh.getParam("peg_h", peg_h);
  priv_nh.getParam("object_h", object_h);
  priv_nh.getParam("object_d", object_d);
  priv_nh.getParam("object_wall_d", object_wall_d);


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

Pose PegTransferDual::poseToCameraFrame(const Pose& pose,
                                        const geometry_msgs::Transform& tr)
{
  Pose ret(pose);
  ret += board_t;
  ret = ret.transform(tr);  // Ori OK
  return ret;
}

Pose PegTransferDual::poseToWorldFrame(const Pose& pose,
                                       const geometry_msgs::Transform& tr)
{
  Pose ret(pose);
  ret = ret.invTransform(tr);
  ret -= board_t;
  return ret;

}


void PegTransferDual::doPegTransfer()
{



  double compress_rate = 0.2;
  int peg_idx_on = 0;
  int peg_idx_to = 6;
  int increment = 1;

  geometry_msgs::Transform e;
  // Read marker transform
  ROS_INFO_STREAM("Waiting for data from vision...");
  e = makeNaN<geometry_msgs::Transform>();
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


  double grasp_translate_x = (object_d / 2.0) - (object_wall_d / 2.0);

  std::vector<Eigen::Vector3d> grasp_translate_world;
  grasp_translate_world.push_back(Eigen::Vector3d(grasp_translate_x, 0.0, object_h));
  grasp_translate_world.push_back(Eigen::Vector3d(-grasp_translate_x, 0.0, object_h));


  std::vector<Eigen::Vector3d> approach_pre_grasp_translate_world;
  approach_pre_grasp_translate_world.push_back(Eigen::Vector3d(grasp_translate_x,
                                                0.0, object_h + 10.0));
  approach_pre_grasp_translate_world.push_back(Eigen::Vector3d(-grasp_translate_x,
                                                0.0, object_h + 10.0));


  std::vector<Eigen::Vector3d> approach_post_grasp_translate_world;
  approach_post_grasp_translate_world.push_back(Eigen::Vector3d(grasp_translate_x,
                                                0.0, (2.0 * object_h) + 10.0));
  approach_post_grasp_translate_world.push_back(Eigen::Vector3d(-grasp_translate_x,
                                                0.0, (2.0 * object_h) + 10.0));

  std::vector<Eigen::Vector3d> approach_pass_grasp_translate_world;
  approach_pass_grasp_translate_world.push_back(Eigen::Vector3d(grasp_translate_x,
                                                0.0, (2.0 * object_h) + 20.0));
  approach_pass_grasp_translate_world.push_back(Eigen::Vector3d(-grasp_translate_x,
                                                0.0, (2.0 * object_h) + 20.0));



  Eigen::Vector3d pass_loc_world(35.0, 35.0, 0.0);

  int pick_arm = 1;
  int place_arm = 0;


  while(ros::ok())
  {

    // Grasp object with pick arm
    ROS_INFO_STREAM("Grasping object on peg " << peg_idx_on << "...");

    Pose grasp_pose_on(peg_positions[peg_idx_on], grasp_ori_world, 0.0);
    grasp_pose_on += grasp_translate_world[pick_arm];
    grasp_pose_on = poseToCameraFrame(grasp_pose_on, e);

    Pose grasp_approach_pose_on(peg_positions[peg_idx_on], grasp_ori_world, 0.0);
    grasp_approach_pose_on += approach_pre_grasp_translate_world[pick_arm];
    grasp_approach_pose_on = poseToCameraFrame(grasp_approach_pose_on, e);

    arms[pick_arm]->grasp(grasp_pose_on, grasp_approach_pose_on, object_wall_d, compress_rate, speed_cartesian, speed_jaw);
    while(!arms[pick_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Bring object to pass location
    ROS_INFO_STREAM("Moving object to pass location...");

    Pose pass_approach_pose_pick(peg_positions[peg_idx_on], grasp_ori_world, 0.0);
    pass_approach_pose_pick += approach_post_grasp_translate_world[pick_arm];
    pass_approach_pose_pick = poseToCameraFrame(pass_approach_pose_pick, e);

    Pose pass_pose_pick(pass_loc_world, grasp_ori_world, 0.0);
    pass_pose_pick += approach_post_grasp_translate_world[pick_arm];
    pass_pose_pick = poseToCameraFrame(pass_pose_pick, e);

    arms[pick_arm] -> place(pass_pose_pick, pass_approach_pose_pick, speed_cartesian);
    while(!arms[pick_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Grasp object with place arm
    ROS_INFO_STREAM("Grasping object at pass location...");

    Pose pass_pose_place(pass_loc_world, grasp_ori_world, 0.0);
    pass_pose_place += approach_post_grasp_translate_world[place_arm];
    pass_pose_place = poseToCameraFrame(pass_pose_place, e);

    Pose pass_approach_pose_place(pass_loc_world, grasp_ori_world, 0.0);
    pass_approach_pose_place += approach_pass_grasp_translate_world[place_arm];
    pass_approach_pose_place = poseToCameraFrame(pass_approach_pose_place, e);

    arms[place_arm]->grasp(pass_pose_place, pass_approach_pose_place, object_wall_d, compress_rate, speed_cartesian, speed_jaw);
    while(!arms[place_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Release with pick arm
    Pose release_approach_pose_pick(peg_positions[peg_idx_to], grasp_ori_world, 0.0);
    release_approach_pose_pick += approach_pass_grasp_translate_world[pick_arm];
    release_approach_pose_pick = poseToCameraFrame(release_approach_pose_pick, e);

    arms[pick_arm] -> release(release_approach_pose_pick, object_wall_d, speed_cartesian, speed_jaw);
    ROS_INFO_STREAM("Release object with pick arm...");
    while(!arms[pick_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }


    // Place object on peg
    ROS_INFO_STREAM("Placing object to peg " << peg_idx_to << "...");

    Pose place_pose_to(peg_positions[peg_idx_to], grasp_ori_world, 0.0);
    place_pose_to += grasp_translate_world[place_arm];
    place_pose_to = poseToCameraFrame(place_pose_to, e);

    Pose place_approach_pose_to(peg_positions[peg_idx_to], grasp_ori_world, 0.0);
    place_approach_pose_to += approach_post_grasp_translate_world[place_arm];
    place_approach_pose_to = poseToCameraFrame(place_approach_pose_to, e);

    arms[place_arm] -> place(place_pose_to, place_approach_pose_to, speed_cartesian);
    while(!arms[place_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }


    // Release with place arm
    Pose release_approach_pose_place(peg_positions[peg_idx_to], grasp_ori_world, 0.0);
    release_approach_pose_place += approach_pre_grasp_translate_world[place_arm];
    release_approach_pose_place = poseToCameraFrame(release_approach_pose_place, e);

    arms[place_arm] -> release(release_approach_pose_place, object_wall_d, speed_cartesian, speed_jaw);
    ROS_INFO_STREAM("Release object with place arm...");
    while(!arms[place_arm] -> isSurgemeDone() && ros::ok())
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
      pick_arm = 0;
      place_arm = 1;

      ros::Duration(1.0).sleep();
    } else if (peg_idx_to == 6) {
      ROS_INFO_STREAM("All pegs transfered");
      peg_idx_on = 0;
      peg_idx_to = 6;
      pick_arm = 1;
      place_arm = 0;

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
    PegTransferDual pnp(nh ,  priv_nh, arm_names);

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







