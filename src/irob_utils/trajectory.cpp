/*
 *  trajectory.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-08
 */

#include "irob_utils/trajectory.hpp"

namespace irob_autosurg {

template <>
Trajectory<Pose>::Trajectory(const irob_autosurg::TrajectoryToolPose& other): dt(other.dt)
{
	for (int i = 0; i < other.poses.size(); i++)
		points.push_back(Pose(other.poses[i]));
}

}
