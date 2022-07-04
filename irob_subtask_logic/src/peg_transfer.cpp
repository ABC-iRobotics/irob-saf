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
  for (int i = 0; i < blocks.size(); i++)
  {
    irob_msgs::GraspObject g_inv;
    g_inv.id = -1;
    blocks[i] = g_inv;
  }
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
  priv_nh.getParam("on_dist_threshold", on_dist_threshold);

  priv_nh.getParam("offs_x", offs_x);
  priv_nh.getParam("offs_y", offs_y);
  priv_nh.getParam("offs_z" , offs_z);


  std::vector<double> param_board_t;
  priv_nh.getParam("board_t", param_board_t);

  std::vector<double> param_peg_positions;
  priv_nh.getParam("peg_positions", param_peg_positions);
  std::vector<double> param_grasp_positions;
  priv_nh.getParam("grasp_positions", param_grasp_positions);
  std::vector<double> param_park_position;
  priv_nh.getParam("park_position", param_park_position);

  for (int i = 0; i < board_t.rows(); i++)
    board_t(i) = param_board_t[i];

  double rows = param_peg_positions.size() / 3;
  for (int i = 0; i < rows; i++)
     peg_positions.push_back(Eigen::Vector3d(
                                param_peg_positions[i * 3],
                                param_peg_positions[(i * 3) + 1],
                                param_peg_positions[(i * 3) + 2]));

  rows = param_grasp_positions.size() / 3;
  for (int i = 0; i < rows; i++)
     grasp_positions.push_back(Eigen::Vector3d(
                                param_grasp_positions[i * 3],
                                param_grasp_positions[(i * 3) + 1],
                                param_grasp_positions[(i * 3) + 2]));

  park_position = Eigen::Vector3d(param_park_position[0],
                             param_park_position[1],
                             param_park_position[2]);

  ROS_INFO_STREAM(
        "Board descriptor read.");
        //: "<< std::endl << board_t << std::endl << peg_positions);
}

bool PegTransfer::isBlockOnPeg(const irob_msgs::GraspObject& g, int p)
{
  if (p < 0 || p > 11)
    return false;
  Eigen::Vector3d block_pos(
          unwrapMsg<geometry_msgs::Point, Eigen::Vector3d>(g.position));
  block_pos = positionToWorldFrame(block_pos, tf_board);
  block_pos(2) = 0.0;
  block_pos(1) = -block_pos(1);
  //ROS_INFO_STREAM("block: " <<  block_pos);
  //ROS_INFO_STREAM("peg: " <<  peg_positions[p]);
  double dist = (block_pos - peg_positions[p]).norm();
  //ROS_INFO_STREAM(p << " dist: " <<  dist);
  return dist <= on_dist_threshold;
}

bool PegTransfer::storeBlock(const irob_msgs::GraspObject& g)
{
  bool found = false;
  for (int i = 0; i < blocks.size(); i++)
  {
    if (isBlockOnPeg(g, i))
    {
      found = true;

      blocks[i] = irob_msgs::GraspObject(g);

    }
  }
  return found;
}

void PegTransfer::administerTransfer(int from, int to)
{
  //blocks[to] = irob_msgs::GraspObject(blocks[from]);
  blocks[from].id = -1;
}

bool PegTransfer::isPegOccupied(int p)
{
  if (p < 0 || p > 11)
    return false;
  return blocks[p].id >= 0;
}

void PegTransfer::storeEnvironment(const irob_msgs::Environment& e)
{
  if (((abs(e.tf_phantom.translation.x) > 0.000001
        || abs(e.tf_phantom.translation.y) > 0.000001
        || abs(e.tf_phantom.translation.z) > 0.000001)))
    tf_board = Eigen::Affine3d(unwrapMsg<geometry_msgs::Transform, Eigen::Affine3d>(e.tf_phantom));

  for (int i = 0; i < e.objects.size(); i++)
  {
    storeBlock(e.objects[i]);
  }

  /*OS_INFO_STREAM("Blocks:");
  for (int i = 0; i < blocks.size(); i++)
  {
    if (isPegOccupied(i))
      ROS_INFO_STREAM(blocks[i]);
  }*/
}

int PegTransfer::choseGraspOnBlock(int p)
{
  if (p < 0 || p > 11 || !isPegOccupied(p))
    return 0;

  int max_d_i = 0;
  double max_d = -1.0;
  for (int i = 0; i < blocks[p].grasp_poses.size(); i++)
  {
    Eigen::Affine3d block_pose(
          unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(blocks[p].grasp_poses[i]));
    Eigen::Vector3d block_pos(block_pose.translation());
    block_pos = positionToWorldFrame(block_pos, tf_board);
    block_pos(2) = 0.0;
    block_pos(1) = -block_pos(1);
    double dist = (block_pos - peg_positions[p]).norm();
    if (dist > max_d)
    {
      max_d = dist;
      max_d_i = i;
    }
  }
  ROS_INFO_STREAM(max_d_i);
  return max_d_i;
}


Eigen::Affine3d PegTransfer::poseToCameraFrame(const Eigen::Affine3d& pose)
{
  Eigen::Affine3d ret(pose);
  ret = tf_board * ret;
  return ret;
}

Eigen::Affine3d PegTransfer::poseToWorldFrame(const Eigen::Affine3d& pose)
{
  Eigen::Affine3d ret(pose);
  ROS_INFO_STREAM(tf_board.translation().x() << ", " <<
                  tf_board.translation().y() << ", " <<
                  tf_board.translation().z());

  ret = tf_board.inverse() * ret;

  return ret;

}

Eigen::Vector3d PegTransfer::positionToCameraFrame(const Eigen::Vector3d& pos,
                                      const Eigen::Affine3d& tr)
{
  Eigen::Vector3d ret(pos);
  ret = Eigen::Translation3d(board_t) * ret;
  ret = tr * ret;  // Ori OK
  return ret;
}

Eigen::Vector3d PegTransfer::positionToWorldFrame(const Eigen::Vector3d& pos,
                                   const Eigen::Affine3d& tr)
{
  Eigen::Vector3d ret(pos);
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
        //grasp_ori_world = poseToWorldFrame(grasp_pose_on, tf_board).rotation();
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



        // Place to new rod
        ROS_INFO_STREAM("Placing object to rod " << peg_idx_to << "...");
        place_grasp_pos_idx = grasp_pos_idx;// % 2;


        Eigen::Affine3d place_approach_pose_on_wf(poseToWorldFrame(grasp_pose_on));
        ROS_INFO_STREAM(place_approach_pose_on_wf.translation().x() << ", " <<
                        place_approach_pose_on_wf.translation().y() << ", " <<
                        place_approach_pose_on_wf.translation().z());
        place_approach_pose_on_wf = offset_peg_h * place_approach_pose_on_wf;


        ROS_INFO_STREAM(place_approach_pose_on_wf.translation().x() << ", " <<
                        place_approach_pose_on_wf.translation().y() << ", " <<
                        place_approach_pose_on_wf.translation().z());

        Eigen::Affine3d place_approach_pose_on_cf(poseToCameraFrame(place_approach_pose_on_wf));
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


        Eigen::Affine3d place_pose_to_cf(poseToCameraFrame(
                                           Eigen::Affine3d(grasp_ori_world * place_pose_to_wf)));
        place_pose_to_cf = offset_cf * place_pose_to_cf;

        Eigen::Affine3d approach_pose_to_cf(poseToCameraFrame(
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







