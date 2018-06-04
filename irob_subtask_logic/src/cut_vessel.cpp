/*
 * 	cut_vessel.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-06-04
 *
 */

#include <irob_subtask_logic/cut_vessel.hpp>


namespace saf {


CutVessel::CutVessel(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                                 std::vector<std::string> arm_names):
  AutosurgAgent(nh, priv_nh, arm_names), vision(nh, "target")
{
  loadBoardDescriptor(priv_nh);
}

CutVessel::~CutVessel()
{
  // TODO Auto-generated destructor stub
}

void CutVessel::loadBoardDescriptor(ros::NodeHandle priv_nh)
{

  priv_nh.getParam("object_h", object_h);
  priv_nh.getParam("object_d", object_d);


  std::vector<double> param_board_t;
  priv_nh.getParam("board_t", param_board_t);

  std::vector<double> param_vessel_ends;
  priv_nh.getParam("vessel_ends", param_vessel_ends);

  for (int i = 0; i < board_t.rows(); i++)
    board_t(i) = param_board_t[i];

  double rows = param_vessel_ends.size() / 3;
  for (int i = 0; i < rows; i++)
    vessel_ends.push_back(Eigen::Vector3d(
                              param_vessel_ends[i * 3],
                            param_vessel_ends[(i * 3) + 1],
        param_vessel_ends[(i * 3) + 2]));

  ROS_INFO_STREAM(
        "Board descriptor read.");
  //: "<< std::endl << board_t << std::endl << peg_positions);
}

Pose CutVessel::poseToCameraFrame(const Pose& pose,
                                        const geometry_msgs::Transform& tr)
{
  Pose ret(pose);
  ret += board_t;
  ret = ret.transform(tr);  // Ori OK
  return ret;
}

Pose CutVessel::poseToWorldFrame(const Pose& pose,
                                       const geometry_msgs::Transform& tr)
{
  Pose ret(pose);
  ret = ret.invTransform(tr);
  ret -= board_t;
  return ret;

}


void CutVessel::doVesselCutting()
{

  double compress_rate = 0.0;

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


  double grasp_translate_y = 25.0;
  double cut_translate_y = 10.0;
  double cut_height = 8.0;

  Eigen::Vector3d  grasp_translate_world (0.0, grasp_translate_y, (object_h));
  Eigen::Vector3d  cut_translate_world (5.0, cut_translate_y, (object_h + cut_height- 8.0));


  Eigen::Vector3d approach_pre_grasp_translate_world(
        0.0, grasp_translate_y, (object_h + 10.0));



  Eigen::Vector3d approach_cut_translate_world (
        5.0,cut_translate_y, (object_h + cut_height + 10.0));

  Eigen::Vector3d grasp_manipulate_vector (0.0,0.0, -cut_height);



  int cut_arm = 1;
  int grasp_arm = 0;

    // Grasp object
    ROS_INFO_STREAM("Grasping vessel");

    Pose grasp_pose(vessel_ends[0], grasp_ori_world, 0.0);
    grasp_pose += grasp_translate_world;
    grasp_pose = poseToCameraFrame(grasp_pose, e);

    Pose grasp_approach_pose(vessel_ends[0], grasp_ori_world, 0.0);
    grasp_approach_pose += approach_pre_grasp_translate_world;
    grasp_approach_pose = poseToCameraFrame(grasp_approach_pose, e);

    arms[grasp_arm]->grasp(grasp_pose, grasp_approach_pose, object_d, compress_rate, speed_cartesian, speed_jaw);

    while(!arms[grasp_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

    // Lift up vessel
    ROS_INFO_STREAM("Lifting vessel");
    arms[grasp_arm]->manipulate(grasp_manipulate_vector, speed_cartesian);
    while(!arms[grasp_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }


    // Cut vessel
    ROS_INFO_STREAM("Cutting vessel");

    Pose cut_pose(vessel_ends[0], grasp_ori_world, 0.0);
    cut_pose += cut_translate_world;
    cut_pose = poseToCameraFrame(cut_pose, e);

    Pose cut_approach_pose(vessel_ends[0], grasp_ori_world, 0.0);
    cut_approach_pose += approach_cut_translate_world;
    cut_approach_pose = poseToCameraFrame(cut_approach_pose, e);

    arms[cut_arm]->cut(cut_pose, cut_approach_pose, object_d, speed_cartesian, speed_jaw);

    while(!arms[cut_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

     ros::Duration(1.0).sleep();

    // Release with cut arm


    arms[cut_arm] -> release(cut_approach_pose, object_d, speed_cartesian, speed_jaw);
    ROS_INFO_STREAM("Release object with cut arm...");
    while(!arms[cut_arm] -> isSurgemeDone() && ros::ok())
    {
      ros::Duration(0.1).sleep();
    }

     ROS_INFO_STREAM("Cutting succeeded");

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
    CutVessel pnp(nh ,  priv_nh, arm_names);

    pnp.doVesselCutting();

    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}







