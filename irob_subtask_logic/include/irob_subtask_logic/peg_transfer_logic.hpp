#ifndef PEG_TRANSFER_LOGIC_HPP
#define PEG_TRANSFER_LOGIC_HPP

/*
 * 	peg_transfer.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2022-08-04
 *
 *  Parent class of the autonomous peg transfer
 *  training exercise.
 *
 */

#include <iostream>
#include <sstream>
#include <vector>
#include <map>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>

#include <irob_utils/tool_pose.hpp>
#include <irob_utils/utils.hpp>
#include <irob_msgs/GraspObject.h>
#include <irob_msgs/Environment.h>
#include <irob_utils/abstract_directions.hpp>
#include <irob_motion/surgeme_client.hpp>
#include <irob_vision_support/vision_client.hpp>

#include <irob_subtask_logic/autosurg_agent.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace saf {

class PegTransferLogic : public AutosurgAgent {


protected:

  VisionClient<irob_msgs::Environment, irob_msgs::Environment> vision;

  Eigen::Vector3d board_t;
  std::vector<Eigen::Vector3d> peg_positions;
  std::vector<Eigen::Vector3d> grasp_positions;
  Eigen::Vector3d park_position;
  double peg_h;
  double object_h;
  double object_d;
  double object_wall_d;
  double on_dist_threshold;

  Eigen::Affine3d tf_board;
  std::array<irob_msgs::GraspObject, 12> blocks;

public:

  PegTransferLogic(ros::NodeHandle nh, ros::NodeHandle priv_nh,
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

  ~PegTransferLogic()
  {
    // TODO Auto-generated destructor stub
  }

  void loadBoardDescriptor(ros::NodeHandle priv_nh)
  {

    priv_nh.getParam("peg_h", peg_h);
    priv_nh.getParam("object_h", object_h);
    priv_nh.getParam("object_d", object_d);
    priv_nh.getParam("object_wall_d", object_wall_d);
    priv_nh.getParam("on_dist_threshold", on_dist_threshold);

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

  bool isBlockOnPeg(const irob_msgs::GraspObject& g, int p)
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

  bool storeBlock(const irob_msgs::GraspObject& g)
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

  void administerTransfer(int from, int to)
  {
    //blocks[to] = irob_msgs::GraspObject(blocks[from]);
    blocks[from].id = -1;
  }

  bool isPegOccupied(int p)
  {
    if (p < 0 || p > 11)
      return false;
    return blocks[p].id >= 0;
  }

  void storeEnvironment(const irob_msgs::Environment& e)
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

  int choseGraspOnBlock(int p)
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


  Eigen::Affine3d poseToCameraFrame(const Eigen::Affine3d& pose)
  {
    Eigen::Affine3d ret(pose);
    ret = tf_board * ret;
    return ret;
  }

  Eigen::Affine3d poseToWorldFrame(const Eigen::Affine3d& pose)
  {
    Eigen::Affine3d ret(pose);
    ROS_INFO_STREAM(tf_board.translation().x() << ", " <<
                    tf_board.translation().y() << ", " <<
                    tf_board.translation().z());

    ret = tf_board.inverse() * ret;

    return ret;

  }

  Eigen::Vector3d positionToCameraFrame(const Eigen::Vector3d& pos,
                                        const Eigen::Affine3d& tr)
  {
    Eigen::Vector3d ret(pos);
    ret = Eigen::Translation3d(board_t) * ret;
    ret = tr * ret;  // Ori OK
    return ret;
  }

  Eigen::Vector3d positionToWorldFrame(const Eigen::Vector3d& pos,
                                     const Eigen::Affine3d& tr)
  {
    Eigen::Vector3d ret(pos);
    ret = tr.inverse() * ret;
    ret = Eigen::Translation3d(board_t).inverse() * ret;
    return ret;

  }

  void doPegTransfer();
  void calibrateOffset();
  void measureAccuracyBlocks();
  void measureAccuracyPegs();

};

}

#endif // PEG_TRANSFER_LOGIC_HPP
