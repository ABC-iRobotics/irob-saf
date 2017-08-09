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
	for (irob_autosurg::ToolPose tp : other.poses)
		points.push_back(Pose(tp));
}


// TODO This method is called frequently, is it effective enough?
template<>
void Trajectory<Pose>::copyToRosTrajectory(
			irob_autosurg::TrajectoryToolPose& ros_tr)
{
  	for (Pose p : points)
  		ros_tr.poses.push_back(p.toRosToolPose());
  	ros_tr.dt = dt;
}

}
