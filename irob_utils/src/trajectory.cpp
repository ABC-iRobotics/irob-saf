/*
 *  trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-08
 */


#include <rclcpp/rclcpp.hpp>
#include <irob_msgs/msg/trajectory_tool_pose.hpp>
#include <irob_msgs/msg/tool_pose.hpp>
#include <vector>
#include <memory>
#include "irob_utils/trajectory.hpp"

namespace saf {

    // Template specialization for ToolPose
    template <>
    Trajectory<ToolPose>::Trajectory(const irob_msgs::msg::TrajectoryToolPose &other)
        : dt(other.dt)
    {
        for (const irob_msgs::msg::ToolPose &tp : other.toolposes)
            points.push_back(ToolPose(tp));
    }

    // TODO: This method is called frequently, is it effective enough?
    template <>
    void Trajectory<ToolPose>::copyToRosTrajectory(
        irob_msgs::msg::TrajectoryToolPose &ros_tr)
    {
        for (const ToolPose &p : points)
            ros_tr.toolposes.push_back(p.toRosToolPose());
        ros_tr.dt = dt;
    }

} // namespace saf