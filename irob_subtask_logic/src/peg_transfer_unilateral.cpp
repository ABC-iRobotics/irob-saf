/*
 * 	peg_transfer.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-01-11
 *
 */

#include <irob_subtask_logic/peg_transfer_unilateral.hpp>


namespace saf {


/**
 * Constructor
 */
PegTransferUnilateral::PegTransferUnilateral(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                         std::vector<std::string> arm_names):
  PegTransferLogic(nh, priv_nh, arm_names)
{

  ROS_INFO_STREAM("d");
  std::vector<double> offset_arm_1;
   ROS_INFO_STREAM("f");
  priv_nh.getParam("offset_arm_2", offset_arm_1);
   ROS_INFO_STREAM("g");
  offs_x = offset_arm_1[0];
  offs_y = offset_arm_1[1];
  offs_z = offset_arm_1[2];
   ROS_INFO_STREAM("h");

  priv_nh.getParam("offset_filename_arm_1", offset_filename_arm_1);
   ROS_INFO_STREAM("i");
  priv_nh.getParam("measurement_filename", measurement_filename);

  ROS_INFO_STREAM("1");


}


/**
 * Destructor
 */
PegTransferUnilateral::~PegTransferUnilateral()
{
  // TODO Auto-generated destructor stub
}



/**
 * Offset calibration
 */
void PegTransferUnilateral::calibrateOffset()
{
  ROS_INFO_STREAM("a");
  irob_msgs::Environment e;
  ROS_INFO_STREAM("Waiting for data from vision...");
  e = makeNaN<irob_msgs::Environment>();

  ROS_INFO_STREAM("b");

  while (((abs(e.tf_phantom.translation.x) < 0.000001
           && abs(e.tf_phantom.translation.y) < 0.000001
           && abs(e.tf_phantom.translation.z) < 0.000001)
          || e.objects.size() == 0 || (isnan(e))) && ros::ok())
  {
    e = vision.getResult();
    storeEnvironment(e);
    ros::Duration(0.1).sleep();
  }

  std::vector<Eigen::Affine3d> waypoints;
  waypoints.push_back(arms[0] -> getPoseCurrent().transform);

  std::vector<Eigen::Translation3d> corner_pos_wf_arr;
  corner_pos_wf_arr.push_back(Eigen::Translation3d(0.0, 0.0, -9.5));
  corner_pos_wf_arr.push_back(Eigen::Translation3d(101.1, 0.0, -9.5));
  corner_pos_wf_arr.push_back(Eigen::Translation3d(101.1, -62.8, -9.5));
  corner_pos_wf_arr.push_back(Eigen::Translation3d(0.0, -62.8, -9.5));

  //Vision data received
  Eigen::Quaternion<double> ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_SIDEWAYS;

  Eigen::Quaternion<double> grasp_ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_FORWARD;

  Eigen::Translation3d offset_sum(0.0, 0.0, 0.0);


  ROS_INFO_STREAM("Starting calibration...");
  for(Eigen::Translation3d corner_pos_wf : corner_pos_wf_arr)
  {
    ROS_INFO_STREAM("Navigating to corner..");

    Eigen::Affine3d corner_pose_cf(poseWf2Cf(
                                     Eigen::Affine3d(grasp_ori_world * corner_pos_wf)));

    arms[0] -> nav_to_pos(corner_pose_cf, speed_cartesian, waypoints);
    while(!arms[0] -> isSurgemeDone() && ros::ok())
    {
      e = vision.getResult();
      storeEnvironment(e);
      ros::Duration(0.1).sleep();
    }

    Eigen::Affine3d uncorr_pose = arms[0] -> getPoseCurrent().transform;


    ROS_INFO_STREAM("Move arm to target position and press Enter...");
    std::cout << "Press enter to continue ...";
    std::cin.get();


    ROS_INFO_STREAM("Saving target position");
    ros::spinOnce();
    Eigen::Affine3d corr_pose = arms[0] -> getPoseCurrent().transform;

    Eigen::Translation3d offset(corr_pose.translation() - uncorr_pose.translation());
    offset_sum = offset * offset_sum;
  }



  ROS_INFO_STREAM("Writing offset to file " << offset_filename_arm_1 << "...");
  std::ofstream offsfile;
  offsfile.open (offset_filename_arm_1);
  offsfile << "offset_arm_1: [" << (offset_sum.x() / 4.0) << ", " << (offset_sum.y() / 4.0)
           << ", " << (offset_sum.z() / 4.0) << "]\n";
  offsfile.close();
  ROS_INFO_STREAM("Offset wrote to file successfully.");

}


/**
 * Accuracy measurement on blocks
 */
void PegTransferUnilateral::measureAccuracyBlocks()
{
  int peg_idx_on = 0;
  int peg_idx_to = 6;
  int increment = 1;
  int grasp_pos_idx = 1;
  int place_grasp_pos_idx = 1;
  double jaw_len = 14.5;

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

  std::vector<Eigen::Affine3d> waypoints;
  waypoints.push_back(arms[0] -> getPoseCurrent().transform);

  //Vision data received

  Eigen::Translation3d offset_cf = Eigen::Translation3d(Eigen::Vector3d(offs_x, offs_y, offs_z));

  Eigen::Translation3d offset_jaw_h =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, jaw_len));

  //Vision data received
    Eigen::Quaternion<double> ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_SIDEWAYS;

    Eigen::Quaternion<double> grasp_ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_FORWARD;

  std::vector<Eigen::Vector3d> err_arr;
  ROS_INFO_STREAM("Size: " << err_arr.size());
  while(ros::ok() && peg_idx_on < 6)
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

        //grasp_pos_idx = choseGraspOnBlock(peg_idx_on);
        Eigen::Affine3d grasp_pose_on(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                                        blocks[peg_idx_on].grasp_poses[grasp_pos_idx]));

        //grasp_ori_world = poseToWorldFrame(grasp_pose_on, tf_board).rotation();
        grasp_pose_on = offset_cf * translationWf2Cf(offset_jaw_h) * grasp_pose_on;


        arms[0] -> nav_to_pos(grasp_pose_on, speed_cartesian, waypoints);
        while(!arms[0] -> isSurgemeDone() && ros::ok())
        {
          e = vision.getResult();
          storeEnvironment(e);
          ros::Duration(0.1).sleep();
        }

        // Correction
        ros::spinOnce();
        Eigen::Affine3d uncorr_pose = arms[0] -> getPoseCurrent().transform;

        ROS_INFO_STREAM("Move arm to target position and press Enter...");
        std::cout << "Press enter to continue ...";
        std::cin.get();


        ROS_INFO_STREAM("Saving target position");
        ros::spinOnce();
        Eigen::Affine3d corr_pose = arms[0] -> getPoseCurrent().transform;
        err_arr.push_back(Eigen::Vector3d(corr_pose.translation() - uncorr_pose.translation()));
        ROS_INFO_STREAM("Size: " << err_arr.size());


        ros::Duration(0.1).sleep();

        administerTransfer(peg_idx_on, peg_idx_to);
        ROS_INFO_STREAM("Block measured.");

        ros::Duration(1.0).sleep();
        peg_idx_on += increment;
        peg_idx_to += increment;

  }

  ROS_INFO_STREAM("Writing measurement to file " << measurement_filename << "...");
  std::ofstream measfile;
  measfile.open(measurement_filename, std::ios_base::app);
  ROS_INFO_STREAM("Size: " << err_arr.size());
  for (Eigen::Vector3d err : err_arr)
    measfile << err.x() << ";" <<  err.y() << ";" <<  err.z() << "\n";
  measfile.close();
  ROS_INFO_STREAM("Measurement wrote to file successfully.");
}



/**
 * Accuracy measurement on pegs
 */
void PegTransferUnilateral::measureAccuracyPegs()
{
  int peg_idx_on = 0;
  int increment = 1;
  double jaw_len = 9.5;
  peg_h = 26.5;

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

  std::vector<Eigen::Affine3d> waypoints;
  waypoints.push_back(arms[0] -> getPoseCurrent().transform);

  //Vision data received

  Eigen::Translation3d offset_cf = Eigen::Translation3d(Eigen::Vector3d(offs_x, offs_y, offs_z));

  Eigen::Translation3d offset_jaw_h =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, -jaw_len));
  Eigen::Translation3d offset_peg_h =
                                Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, -peg_h));

  //Vision data received
    Eigen::Quaternion<double> ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_SIDEWAYS;

    Eigen::Quaternion<double> grasp_ori_world =
          BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
          DOWN_FORWARD;

  std::vector<Eigen::Vector3d> err_arr;
  ROS_INFO_STREAM("Size: " << err_arr.size());
  while(ros::ok() && peg_idx_on < 12)
  {
      e = vision.getResult();
      storeEnvironment(e);


        ROS_INFO_STREAM("Touchuing peg " << peg_idx_on << "...");

        Eigen::Translation3d peg_pos_wf(peg_positions[peg_idx_on]);
        peg_pos_wf = offset_jaw_h * offset_peg_h * peg_pos_wf;


        Eigen::Affine3d peg_pose_cf(poseWf2Cf(Eigen::Affine3d(grasp_ori_world *peg_pos_wf)));
        peg_pose_cf = offset_cf * peg_pose_cf;
        //grasp_ori_world = poseToWorldFrame(grasp_pose_on, tf_board).rotation();



        arms[0] -> nav_to_pos(peg_pose_cf, speed_cartesian, waypoints);
        while(!arms[0] -> isSurgemeDone() && ros::ok())
        {
          e = vision.getResult();
          storeEnvironment(e);
          ros::Duration(0.1).sleep();
        }

        // Correction
        ros::spinOnce();
        Eigen::Affine3d uncorr_pose = arms[0] -> getPoseCurrent().transform;

        ROS_INFO_STREAM("Move arm to target position and press Enter...");
        std::cout << "Press enter to continue ...";
        std::cin.get();


        ROS_INFO_STREAM("Saving target position");
        ros::spinOnce();
        Eigen::Affine3d corr_pose = arms[0] -> getPoseCurrent().transform;
        err_arr.push_back(Eigen::Vector3d(corr_pose.translation() - uncorr_pose.translation()));
        ROS_INFO_STREAM("Size: " << err_arr.size());


        ros::Duration(0.1).sleep();
        ROS_INFO_STREAM("Block measured.");

        ros::Duration(1.0).sleep();
        peg_idx_on += increment;

  }

  ROS_INFO_STREAM("Writing measurement to file " << measurement_filename << "...");
  std::ofstream measfile;
  measfile.open(measurement_filename, std::ios_base::app);
  ROS_INFO_STREAM("Size: " << err_arr.size());
  for (Eigen::Vector3d err : err_arr)
    measfile << err.x() << ";" <<  err.y() << ";" <<  err.z() << "\n";
  measfile.close();
  ROS_INFO_STREAM("Measurement wrote to file successfully.");
}



/**
 * Execute peg transfer
 */
void PegTransferUnilateral::doPegTransfer()
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

        //grasp_pos_idx = choseGraspOnBlock(peg_idx_on);
        Eigen::Affine3d grasp_pose_on(unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(
                                                        blocks[peg_idx_on].grasp_poses[grasp_pos_idx]));
        //grasp_ori_world = poseToWorldFrame(grasp_pose_on, tf_board).rotation();
        Eigen::Affine3d place_approach_pose_on_wf(poseCf2Wf(grasp_pose_on));
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

        ROS_INFO_STREAM("Grasped object on rod " << peg_idx_on << "...");
        ros::Duration(0.1).sleep();



        // Place to new peg
        ROS_INFO_STREAM("Placing object to peg " << peg_idx_to << "...");
        place_grasp_pos_idx = grasp_pos_idx;// % 2;



        ROS_INFO_STREAM(place_approach_pose_on_wf.translation().x() << ", " <<
                        place_approach_pose_on_wf.translation().y() << ", " <<
                        place_approach_pose_on_wf.translation().z());
        place_approach_pose_on_wf = offset_peg_h * place_approach_pose_on_wf;


        ROS_INFO_STREAM(place_approach_pose_on_wf.translation().x() << ", " <<
                        place_approach_pose_on_wf.translation().y() << ", " <<
                        place_approach_pose_on_wf.translation().z());

        Eigen::Affine3d place_approach_pose_on_cf(poseWf2Cf(place_approach_pose_on_wf));
        place_approach_pose_on_cf = offset_cf * place_approach_pose_on_cf;

        std::vector<Eigen::Affine3d> waypoints;
        waypoints.push_back(place_approach_pose_on_cf);
        ROS_INFO_STREAM(place_approach_pose_on_cf.translation().x() << ", " <<
                        place_approach_pose_on_cf.translation().y() << ", " <<
                        place_approach_pose_on_cf.translation().z());


        Eigen::Translation3d place_pose_to_wf(peg_positions[peg_idx_to]);
        place_pose_to_wf
            = Eigen::Translation3d(grasp_positions[place_grasp_pos_idx]) * offset_block_h.inverse()
                                                                                      * place_pose_to_wf;

        Eigen::Translation3d approach_pose_to_wf(place_pose_to_wf);
        approach_pose_to_wf =  offset_peg_h.inverse() * approach_pose_to_wf;


        Eigen::Affine3d place_pose_to_cf(poseWf2Cf(
                                           Eigen::Affine3d(grasp_ori_world * place_pose_to_wf)));
        place_pose_to_cf = offset_cf * place_pose_to_cf;

        Eigen::Affine3d approach_pose_to_cf(poseWf2Cf(
                                           Eigen::Affine3d(grasp_ori_world * approach_pose_to_wf)));
        approach_pose_to_cf = offset_cf * approach_pose_to_cf;


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
  ros::init(argc, argv, "peg_transfer_unilateral");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::vector<std::string> arm_names;
  priv_nh.getParam("arm_names", arm_names);

  std::string mode;
  priv_nh.getParam("mode", mode);


  // Start autonomous agent
  try {
    PegTransferUnilateral pnp(nh ,  priv_nh, arm_names);
    ROS_INFO_STREAM("2");

    if (mode == "execution")
      pnp.doPegTransfer();
    else if (mode == "calibration") {
      ROS_INFO_STREAM("3");
      pnp.calibrateOffset();
    }else if (mode == "acc_blocks")
      pnp.measureAccuracyBlocks();
    else if (mode == "acc_pegs")
      pnp.measureAccuracyPegs();
    else
      ROS_INFO_STREAM("Mode invalid. Valid modes are: [execution, calibration, acc_blocks, acc_pegs].");


    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}







