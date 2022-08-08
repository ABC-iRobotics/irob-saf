/*
 * 	peg_transfer.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-01-11
 *
 */

#include <irob_subtask_logic/peg_transfer_bilateral.hpp>


namespace saf {


PegTransferBilateral::PegTransferBilateral(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                         std::vector<std::string> arm_names):
  PegTransferLogic(nh, priv_nh, arm_names)
{
  loadBoardDescriptor(priv_nh);
  for (int i = 0; i < blocks.size(); i++)
  {
    irob_msgs::GraspObject g_inv;
    g_inv.id = -1;
    blocks[i] = g_inv;
  }
}

PegTransferBilateral::~PegTransferBilateral()
{
  // TODO Auto-generated destructor stub
}



void PegTransferBilateral::doPegTransfer()
{



  double compress_rate = 0.2;
  int peg_idx_on = 0;
  int peg_idx_to = 6;
  int increment = 1;
  int grasp_pos_idx = 1;
  int place_grasp_pos_idx = 1;

  irob_msgs::Environment e;
  // Read marker transform
  ROS_INFO_STREAM("Waiting for data from vision...");
  e = makeNaN<irob_msgs::Environment>();

  while (((abs(e.tf_phantom.translation.x) < 0.000001
           && abs(e.tf_phantom.translation.y) < 0.000001
           && abs(e.tf_phantom.translation.z) < 0.000001)
          || e.objects.size() == 0 || (isnan(e))) && ros::ok())
  {
    e = vision.getResult();
    storeEnvironment(e);
    ros::Duration(0.1).sleep();
  }

  //Vision data received

  Eigen::Translation3d offset_cf = Eigen::Translation3d(Eigen::Vector3d(offs_x, offs_y, offs_z));

  Eigen::Translation3d offset_peg_h =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, peg_h));

  Eigen::Translation3d offset_block_h =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, object_h));

  //Vision data received
    Eigen::Quaternion<double> ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_SIDEWAYS;

    Eigen::Quaternion<double> grasp_ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_FORWARD;

  while(ros::ok())
  {
    do
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    } while(!isPegOccupied(peg_idx_on));

    //ROS_INFO_STREAM("blocks[peg_idx_on]: "<<   blocks[peg_idx_on]);

    // Grasp object
        ROS_INFO_STREAM("Grasping object on rod " << peg_idx_on << "...");
        Eigen::Affine3d tf_board(unwrapMsg<geometry_msgs::Transform, Eigen::Affine3d>(e.tf_phantom));

        //grasp_pos_idx = choseGraspOnBlock(peg_idx_on);
        Eigen::Affine3d grasp_pose_on(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                                        blocks[peg_idx_on].grasp_poses[grasp_pos_idx]));
        grasp_ori_world = poseToWorldFrame(grasp_pose_on).rotation();
        grasp_pose_on = offset_cf * grasp_pose_on;



        Eigen::Affine3d grasp_approach_pose_on(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                                 blocks[peg_idx_on].approach_poses[grasp_pos_idx]));

        grasp_approach_pose_on = offset_cf * grasp_approach_pose_on;

        arms[0]->grasp(grasp_pose_on, grasp_approach_pose_on, object_wall_d, compress_rate, speed_cartesian, speed_jaw);
        while(!arms[0] -> isSurgemeDone() && ros::ok())
        {
          e = vision.getResult();
          storeEnvironment(e);
          ros::Duration(0.1).sleep();
        }
        ROS_INFO_STREAM(grasp_pose_on.translation().x() << ", " <<
                        grasp_pose_on.translation().y() << ", " <<
                        grasp_pose_on.translation().z());

        ROS_INFO_STREAM("Grasped object on peg " << peg_idx_on << "...");
        ros::Duration(0.1).sleep();



        // Navigate to Pass position
        ROS_INFO_STREAM("Navigating to pass position...");
        place_grasp_pos_idx = grasp_pos_idx + 2;// % 2;


        Eigen::Affine3d place_approach_pose_on_wf(poseToWorldFrame(grasp_pose_on));
        place_approach_pose_on_wf = offset_peg_h * place_approach_pose_on_wf;


        Eigen::Affine3d place_approach_pose_on_cf(poseToCameraFrame(place_approach_pose_on_wf));
        place_approach_pose_on_cf = offset_cf * place_approach_pose_on_cf;

        std::vector<Eigen::Affine3d> waypoints;
        waypoints.push_back(place_approach_pose_on_cf);


        Eigen::Translation3d place_pose_to_wf(peg_positions[peg_idx_to]);
        place_pose_to_wf
            = Eigen::Translation3d(grasp_positions[place_grasp_pos_idx]) * offset_block_h.inverse()
                                                                                      * place_pose_to_wf;

        Eigen::Translation3d approach_pose_to_wf(place_pose_to_wf);
        approach_pose_to_wf =  offset_peg_h.inverse() * approach_pose_to_wf;


        Eigen::Affine3d place_pose_to_cf(poseToCameraFrame(
                                           Eigen::Affine3d(grasp_ori_world * place_pose_to_wf)));
        place_pose_to_cf = offset_cf * place_pose_to_cf;

        Eigen::Affine3d approach_pose_to_cf(poseToCameraFrame(
                                           Eigen::Affine3d(grasp_ori_world * approach_pose_to_wf)));
        approach_pose_to_cf = offset_cf * approach_pose_to_cf;

        //Transfer location
        Eigen::Translation3d pass_arm_1_pose_wf(place_approach_pose_on_wf.translation());
        pass_arm_1_pose_wf = approach_pose_to_wf* pass_arm_1_pose_wf;



        Eigen::Translation3d pass_arm_1_approach_pose_wf(pass_arm_1_pose_wf);
        pass_arm_1_approach_pose_wf =  offset_peg_h.inverse() * pass_arm_1_approach_pose_wf;

        arms[0] -> place(place_pose_to_cf, approach_pose_to_cf, speed_cartesian, waypoints);
        //arms[0]->grasp(peg_top_cf, peg_top_cf, object_wall_d, compress_rate, speed_cartesian, speed_jaw);
        while(!arms[0] -> isSurgemeDone() && ros::ok())
        {
          e = vision.getResult();
          storeEnvironment(e);
          ros::Duration(0.1).sleep();
        }


        ros::Duration(0.1).sleep();

        // Release
        Eigen::Affine3d release_approach_pose_to(approach_pose_to_cf);

        arms[0] -> release(release_approach_pose_to, object_wall_d, speed_cartesian, speed_jaw);
        ROS_INFO_STREAM("Release object...");
        while(!arms[0] -> isSurgemeDone() && ros::ok())
        {
          e = vision.getResult();
          storeEnvironment(e);
          ros::Duration(0.1).sleep();
        }
        administerTransfer(peg_idx_on, peg_idx_to);
        ROS_INFO_STREAM("Peg-transfer subtask done");



         // Park
         /*Eigen::Translation3d pask_pose_wf(park_position);
         Eigen::Affine3d pask_pose_cf(poseToCameraFrame(
                                            Eigen::Affine3d(grasp_ori_world * pask_pose_wf)));
         pask_pose_cf = offset_cf * pask_pose_cf;

         arms[0] -> nav_to_pos(pask_pose_cf, speed_cartesian);
         ROS_INFO_STREAM("Navigating to parking position...");
         while(!arms[0] -> isSurgemeDone() && ros::ok())
         {
           e = vision.getResult();
           storeEnvironment(e);
           ros::Duration(0.1).sleep();
         }*/
         ros::Duration(1.0).sleep();
         peg_idx_on += increment;
         peg_idx_to += increment;

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
    PegTransferBilateral pnp(nh ,  priv_nh, arm_names);

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







