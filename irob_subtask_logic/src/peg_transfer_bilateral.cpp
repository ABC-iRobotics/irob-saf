/*
 * 	peg_transfer.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-01-11
 *
 */

#include <irob_subtask_logic/peg_transfer_bilateral.hpp>


namespace saf {


/**
 * Constructor
 */
PegTransferBilateral::PegTransferBilateral(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                         std::vector<std::string> arm_names):
  PegTransferLogic(nh, priv_nh, arm_names)
{

  std::vector<double> offset_arm_1;
  priv_nh.getParam("offset_arm_1", offset_arm_1);
  offs_1_x = offset_arm_1[0];
  offs_1_y = offset_arm_1[1];
  offs_1_z = offset_arm_1[2];

  std::vector<double> offset_arm_2;
  priv_nh.getParam("offset_arm_2", offset_arm_2);
  offs_2_x = offset_arm_2[0];
  offs_2_y = offset_arm_2[1];
  offs_2_z = offset_arm_2[2];





}


/**
 * Destructor
 */
PegTransferBilateral::~PegTransferBilateral()
{
  // TODO Auto-generated destructor stub
}

/**
 * Offset calibration
 */
void PegTransferBilateral::calibrateOffset()
{



}


/**
 * Accuracy measurement on blocks
 */
void PegTransferBilateral::measureAccuracyBlocks()
{

}


/**
 * Accuracy measurement on pegs
 */
void PegTransferBilateral::measureAccuracyPegs()
{

}



/**
 * Execute peg transfer
 */
void PegTransferBilateral::doPegTransfer()
{



  double compress_rate = 0.1;
  int peg_idx_on = 0;
  int peg_idx_to = 6;
  int increment = 1;
  int grasper_grasp_pos_idx = 0;
  int placer_grasp_pos_idx = 2;

  int grasper_arm_idx = 0;
  int placer_arm_idx = 1;

   Eigen::Affine3d grasper_pass_pose_cf;
  Eigen::Affine3d placer_pass_pose_cf;
  Eigen::Affine3d grasper_pass_release_pose_cf;
  Eigen::Affine3d placer_pass_approach_pose_cf;

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

  ROS_INFO_STREAM("1");

  //Vision data received

  std::vector<Eigen::Translation3d> offset_cf;
  offset_cf.push_back(Eigen::Translation3d(Eigen::Vector3d(
                                    offs_2_x, offs_2_y, offs_2_z)));
  offset_cf.push_back(Eigen::Translation3d(Eigen::Vector3d(
                                    offs_1_x, offs_1_y, offs_1_z)));

  Eigen::Translation3d offset_peg_h =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, peg_h));

  Eigen::Translation3d offset_tool_l =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, tool_l));

  Eigen::Translation3d offset_sag_h=
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, sag_h));

  Eigen::Translation3d offset_block_h =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, object_h));

  //Vision data received
    Eigen::Quaternion<double> ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_SIDEWAYS;

    std::vector<Eigen::Quaternion<double>> grasp_ori_world;
    grasp_ori_world.push_back(BaseOrientations<CoordinateFrame::ROBOT,
                              Eigen::Quaternion<double>>::DOWN_FORWARD);
    grasp_ori_world.push_back(Eigen::AngleAxisd(-0.33*M_PI, Eigen::Vector3d::UnitZ()) *
                              (BaseOrientations<CoordinateFrame::ROBOT,
                               Eigen::Quaternion<double>>::DOWN_FORWARD));
    grasp_ori_world.push_back(Eigen::AngleAxisd(0.33*M_PI, Eigen::Vector3d::UnitZ()) *
                              (BaseOrientations<CoordinateFrame::ROBOT,
                               Eigen::Quaternion<double>>::DOWN_FORWARD));
  while(ros::ok())
  {
     ROS_INFO_STREAM("2");
    do
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    } while(!isPegOccupied(peg_idx_on));
     ROS_INFO_STREAM("3");


    // Calculate poses


      //Grasp

    Eigen::Affine3d tf_board(unwrapMsg<geometry_msgs::Transform, Eigen::Affine3d>(e.tf_phantom));

    //grasp_pos_idx = choseGraspOnBlock(peg_idx_on);

        // grasper_grasp_pose
    placer_grasp_pos_idx = (grasper_grasp_pos_idx + 3) % 6;
    Eigen::Affine3d grasper_grasp_pose(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                                    blocks[peg_idx_on].grasp_poses[grasper_grasp_pos_idx]));
    Eigen::Quaternion<double> grasped_grasp_ori_world(poseCf2Wf(grasper_grasp_pose).rotation());

    Eigen::Affine3d grasper_grasp_leave_pose_wf(poseCf2Wf(grasper_grasp_pose));

    grasper_grasp_pose = offset_cf[grasper_arm_idx] * grasper_grasp_pose;


        // grasper_grasp_approach_pose
    Eigen::Affine3d grasper_grasp_approach_pose(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                             blocks[peg_idx_on].approach_poses[grasper_grasp_pos_idx]));
    grasper_grasp_approach_pose = offset_cf[grasper_arm_idx] * grasper_grasp_approach_pose;

        // grasper_grasp_leave_pose_wf

    grasper_grasp_leave_pose_wf = offset_peg_h * grasper_grasp_leave_pose_wf;
    Eigen::Affine3d grasper_grasp_leave_pose_cf(poseWf2Cf(grasper_grasp_leave_pose_wf));
    grasper_grasp_leave_pose_cf = offset_cf[grasper_arm_idx] * grasper_grasp_leave_pose_cf;



      //Place

        // placer_place_pose_cf
    Eigen::Translation3d placer_place_pose_wf(Eigen::Vector3d(peg_positions[peg_idx_to].x()
                                              ,-peg_positions[peg_idx_to].y()
                                              ,peg_positions[peg_idx_to].z()));
    placer_place_pose_wf
        = Eigen::Translation3d(Eigen::Vector3d( grasp_positions[placer_grasp_pos_idx].x()
                                               ,- grasp_positions[placer_grasp_pos_idx].y()
                                               , grasp_positions[placer_grasp_pos_idx].z()))
                               * offset_block_h * placer_place_pose_wf;
    Eigen::Affine3d placer_place_pose_cf(poseWf2Cf(
                                       Eigen::Affine3d(placer_place_pose_wf * grasp_ori_world[1])));
    placer_place_pose_cf = offset_cf[placer_arm_idx] * placer_place_pose_cf;

      // placer_place_approach_pose_wf
    Eigen::Translation3d placer_place_approach_pose_wf(placer_place_pose_wf);
    placer_place_approach_pose_wf =  offset_peg_h * placer_place_approach_pose_wf;
    Eigen::Affine3d placer_place_approach_pose_cf(poseWf2Cf(
                                       Eigen::Affine3d(placer_place_approach_pose_wf * grasp_ori_world[1])));
    placer_place_approach_pose_cf = offset_cf[placer_arm_idx] * placer_place_approach_pose_cf;


      // Release

        // placer_release_approach_pose_cf
    Eigen::Affine3d placer_release_approach_pose_cf(placer_place_approach_pose_cf);


    if(peg_idx_on < 3) {


      //Transfer location, ori standard

        // grasper_pass_pose_cf
    Eigen::Translation3d grasper_pass_pose_wf(grasper_grasp_leave_pose_wf.translation());
    grasper_pass_pose_wf = placer_place_approach_pose_wf * grasper_pass_pose_wf;
    grasper_pass_pose_wf = Eigen::Translation3d((Eigen::Scaling(0.5) * grasper_pass_pose_wf).translation());
    grasper_pass_pose_cf = Eigen::Affine3d (poseWf2Cf(
                                       Eigen::Affine3d(grasper_pass_pose_wf * grasp_ori_world[0] )));
    grasper_pass_pose_cf = offset_cf[grasper_arm_idx] * grasper_pass_pose_cf;

        // pass_pose
    Eigen::Translation3d pass_pose(grasper_pass_pose_wf);
    pass_pose = Eigen::Translation3d(Eigen::Vector3d( grasp_positions[grasper_grasp_pos_idx].x()
                                                      ,- grasp_positions[grasper_grasp_pos_idx].y()
                                                      , grasp_positions[grasper_grasp_pos_idx].z())).inverse() * pass_pose;

        // placer_pass_pose_cf
    Eigen::Translation3d placer_pass_pose_wf(pass_pose);
    placer_pass_pose_wf = Eigen::Translation3d(Eigen::Vector3d( grasp_positions[placer_grasp_pos_idx].x()
                                                                ,- grasp_positions[placer_grasp_pos_idx].y()
                                                                , grasp_positions[placer_grasp_pos_idx].z())) * placer_pass_pose_wf;
    placer_pass_pose_wf =  offset_sag_h.inverse() * placer_pass_pose_wf;
    placer_pass_pose_cf =  Eigen::Affine3d(poseWf2Cf(
                                       Eigen::Affine3d(placer_pass_pose_wf * grasp_ori_world[1])));
    placer_pass_pose_cf = offset_cf[placer_arm_idx] * placer_pass_pose_cf;

        // grasper_pass_release_pose_cf
    Eigen::Translation3d grasper_pass_release_pose_wf(grasper_pass_pose_wf);
    grasper_pass_release_pose_wf =  offset_tool_l * grasper_pass_release_pose_wf;
    grasper_pass_release_pose_cf = Eigen::Affine3d(poseWf2Cf(
                                           Eigen::Affine3d(grasper_pass_release_pose_wf * grasp_ori_world[0])));
    grasper_pass_release_pose_cf = offset_cf[grasper_arm_idx] * grasper_pass_release_pose_cf;

      // placer_pass_approach_pose_cf
    Eigen::Translation3d placer_pass_approach_pose_wf(placer_pass_pose_wf);
    placer_pass_approach_pose_wf =  offset_tool_l * placer_pass_approach_pose_wf;
    placer_pass_approach_pose_cf = Eigen::Affine3d(poseWf2Cf(
                                       Eigen::Affine3d(placer_pass_approach_pose_wf * grasp_ori_world[1])));
    placer_pass_approach_pose_cf = offset_cf[placer_arm_idx] * placer_pass_approach_pose_cf;

    }




    // Grasp object with grasper arm
    ROS_INFO_STREAM("Grasping object on rod " << peg_idx_on << "...");

    arms[grasper_arm_idx]
            -> grasp(grasper_grasp_pose, grasper_grasp_approach_pose,
                    object_wall_d, compress_rate, speed_cartesian, speed_jaw);

    while(!arms[grasper_arm_idx] -> isSurgemeDone() && ros::ok())
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Grasped object on peg " << peg_idx_on << "...");
    ros::Duration(0.1).sleep();



    // Navigate grasper arm to pass location
    ROS_INFO_STREAM("Navigating grasper arm to transfer location...");
    std::vector<Eigen::Affine3d> waypoints_grasp_pass;
    waypoints_grasp_pass.push_back(grasper_grasp_leave_pose_cf);

    arms[grasper_arm_idx]
            -> nav_to_pos(grasper_pass_pose_cf, speed_cartesian, waypoints_grasp_pass);

    while(!arms[grasper_arm_idx] -> isSurgemeDone() && ros::ok())
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Grasper arm at transfer location.");
    ros::Duration(0.1).sleep();



    // Grasp object with placer
    ROS_INFO_STREAM("Grasping object on transfer...");

    arms[placer_arm_idx]
            -> grasp(placer_pass_pose_cf, placer_pass_approach_pose_cf,
                    object_wall_d, compress_rate, speed_cartesian, speed_jaw);

    while(!arms[placer_arm_idx] -> isSurgemeDone() && ros::ok())
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Grasped object on transfer.");
    ros::Duration(0.1).sleep();


    // Release with grasper
    ROS_INFO_STREAM("Releaseing with grasper arm...");

    arms[grasper_arm_idx]
            -> release(grasper_pass_release_pose_cf, object_wall_d,
                       speed_cartesian, speed_jaw);
    while(!arms[grasper_arm_idx] -> isSurgemeDone() && ros::ok())
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Object released with grasper.");



    // Place with placer
    ROS_INFO_STREAM("Placing object to peg " << peg_idx_to << "...");
    arms[placer_arm_idx]
        -> place(placer_place_pose_cf, placer_place_approach_pose_cf,
                 speed_cartesian);
    while(!arms[placer_arm_idx] -> isSurgemeDone() && ros::ok())
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    }



    // Release with placer
    ROS_INFO_STREAM("Releaseing with placer arm...");

    arms[placer_arm_idx]
            -> release(placer_release_approach_pose_cf, object_wall_d,
                       speed_cartesian, speed_jaw);
    while(!arms[placer_arm_idx] -> isSurgemeDone() && ros::ok())
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Object released with placer.");

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







