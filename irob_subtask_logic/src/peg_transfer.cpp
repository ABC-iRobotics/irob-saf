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
  AutosurgAgent(nh, arm_names), vision(nh, "target")
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


  std::vector<double> param_board_t;
  priv_nh.getParam("board_t", param_board_t);

  std::vector<double> param_peg_positions;
  priv_nh.getParam("peg_positions", param_peg_positions);

  for (int i = 0; i < board_t.rows(); i++)
    board_t(i) = param_board_t[i];

  double rows = param_peg_positions.size() / 3;
  for (int i = 0; i < rows; i++)
     peg_positions.push_back(Eigen::Vector3d(
                                param_peg_positions[i * rows],
                                param_peg_positions[(i * rows) + 1],
                                param_peg_positions[(i * rows) + 2]));

  ROS_INFO_STREAM(
        "Board descriptor read.");
        //: "<< std::endl << board_t << std::endl << peg_positions);
}

Pose PegTransfer::poseToCameraFrame(const Pose& pose,
                                      const geometry_msgs::Transform& tr)
{
  Pose ret(pose);
  ret -= board_t;
  ret = ret.transform(tr);
  return ret;
}

Pose PegTransfer::poseToWorldFrame(const Pose& pose,
                                   const geometry_msgs::Transform& tr)
{
  Pose ret(pose);
  ret = ret.invTransform(tr);
  ret += board_t;
  return ret;

}


void PegTransfer::doPegTransfer()
{
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

  Eigen::Quaternion<double> dist_ori_world(ori_world);
  Eigen::Quaternion<double> grasp_ori_world(ori_world);

  Pose dp_world(Eigen::Vector3d(90.0, 0.0, 70.0), ori_world, 0.0);

  double grasp_translate_x = (object_d / 2.0) - (object_wall_d / 2.0);

  Eigen::Vector3d grasp_translate_world(grasp_translate_x, 0.0, object_h);

  Eigen::Vector3d approach_pre_grasp_translate_world(grasp_translate_x,
                                                     0.0, object_h + 10.0);

  Eigen::Vector3d approach_post_grasp_translate_world(grasp_translate_x,
                                                      0.0, object_h + peg_h + 10.0);


  double compress_rate = 0.2;
  int peg_idx_on = 0;
  int peg_idx_to = 6;
  int increment = 1;
  double speed_cartesian = 30.0;
  double speed_jaw = 30.0;


  while(ros::ok())
  {

    // Grasp object
    ROS_INFO_STREAM("Grasping object on rod " << peg_idx_on << "...");

    Pose grasp_pose_on(peg_positions[peg_idx_on], grasp_ori_world, 0.0);
    grasp_pose_on += grasp_translate_world;
    grasp_pose_on = poseToCameraFrame(grasp_pose_on, e);

    Pose approach_pose_on(peg_positions[peg_idx_on], grasp_ori_world, 0.0);
    approach_pose_on += approach_pre_grasp_translate_world;
    approach_pose_on = poseToCameraFrame(approach_pose_on, e);

    arms[0]->grasp(grasp_pose_on, approach_pose_on, object_wall_d, compress_rate, speed_cartesian, speed_jaw);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Place to new rod
    ROS_INFO_STREAM("Placing object to rod " << peg_idx_to << "...");

    Pose grasp_pose_to(peg_positions[peg_idx_to], grasp_ori_world, 0.0);
    grasp_pose_to += grasp_translate_world;
    grasp_pose_to = poseToCameraFrame(grasp_pose_to, e);

    Pose approach_pose_to(peg_positions[peg_idx_to], grasp_ori_world, 0.0);
    approach_pose_to += approach_post_grasp_translate_world;
    approach_pose_to = poseToCameraFrame(approach_pose_to, e);

    std::vector<Pose> waypoints;
    waypoints.push_back(approach_pose_on);
    //waypoints.push_back(dist_pose);
    arms[0] -> place(grasp_pose_to, approach_pose_to, speed_cartesian, waypoints);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }


    // Release
    arms[0] -> release(approach_pose_to, object_wall_d, speed_cartesian, speed_jaw);
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







