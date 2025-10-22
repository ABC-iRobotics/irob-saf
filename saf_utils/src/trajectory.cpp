/*
 *  trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-08
 */


#include <rclcpp/rclcpp.hpp>
#include <saf_msgs/msg/trajectory_tool_pose.hpp>
#include <saf_msgs/msg/tool_pose.hpp>
#include <vector>
#include <memory>
#include "saf_utils/trajectory.hpp"

namespace saf {

    // Template specialization for ToolPose
    template <>
    Trajectory<ToolPose>::Trajectory(const saf_msgs::msg::TrajectoryToolPose &other)
        : dt(other.dt)
    {
        for (const saf_msgs::msg::ToolPose &tp : other.toolposes)
            points.push_back(ToolPose(tp));
    }

    // TODO: This method is called frequently, is it effective enough?
    template <>
    void Trajectory<ToolPose>::copyToRosTrajectory(
        saf_msgs::msg::TrajectoryToolPose &ros_tr)
    {
        for (const ToolPose &p : points)
            ros_tr.toolposes.push_back(p.toRosToolPose());
        ros_tr.dt = dt;
    }

} // namespace saf