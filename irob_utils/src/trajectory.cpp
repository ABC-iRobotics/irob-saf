/*
 *  trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-08
 */

#include <irob_utils/trajectory.hpp>

namespace saf {

template <>
Trajectory<ToolPose>::Trajectory(const irob_msgs::TrajectoryToolPose& other): dt(other.dt)
{
  for (irob_msgs::ToolPose tp : other.toolposes)
		points.push_back(ToolPose(tp));
}


// TODO This method is called frequently, is it effective enough?
template<>
void Trajectory<ToolPose>::copyToRosTrajectory(
			irob_msgs::TrajectoryToolPose& ros_tr)
{
  	for (ToolPose p : points)
      ros_tr.toolposes.push_back(p.toRosToolPose());
  	ros_tr.dt = dt;
}

}
