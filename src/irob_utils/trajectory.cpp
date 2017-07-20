/*
 *  trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-08
 */

#include "irob_utils/trajectory.hpp"

namespace ias {

template <>
Trajectory<Pose>::Trajectory(const irob_autosurg::TrajectoryToolPose& other): dt(other.dt)
{
	for (int i = 0; i < other.poses.size(); i++)
		points.push_back(Pose(other.poses[i]));
}

// TODO This method is called frequently, is it effective enough?
template<>
void Trajectory<Pose>copyToRosTrajectory(
			irob_autosurg::ToolPoseTrajectory& ros_tr);
{
  	for (int i = 0; i < size(); i++)
  		ros_tr.poses.push_back(points[i].toRosToolPose());
  	ros_tr.dt = dt;
}

}
