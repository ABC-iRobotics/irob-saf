/*
 * trajectory_factory.hpp
 *
 *  Created on: 2016. okt. 25.
 *      Author: tamas
 */

#ifndef TRAJECTORY_FACTORY_HPP_
#define TRAJECTORY_FACTORY_HPP_

#include <iostream>
#include <vector>
#include "trajectory.hpp"
#include "pose.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

class TrajectoryFactory
{

	private:
		TrajectoryFactory() {}
	public:
		template <class P>
		static Trajectory<P>* linearTrajectory(P start,P end, 
											double T, double dt)
		{
			Trajectory<P>* tr = new Trajectory<P>(dt);
			int N = (int)round(T / dt)+1;
			for (int i = 0; i < N; i++)
			{
				P p = ((start*((N-i)/(double)N)) +(end*(i/(double)N)));
				tr->addPoint(p);
			}
			tr->addPoint(end);
			return tr;
		}
		
		template <class P>
		static Trajectory<P>* acceleratingTrajectory(P start, P stepStep,
											P maxStep, double dt)
		{
			Trajectory<P>* tr = new Trajectory<P>(dt);
			P pos = start;
			P step = 0.0;
			while (step < maxStep)
			{
				pos += step;
				tr->addPoint(pos);
				step += stepStep;
			}
			return tr;
		}
		
		
		
		static Trajectory<Eigen::Vector3d>* circleTrajectoryHorizontal(
			Eigen::Vector3d start, 
			double toAngle, Eigen::Vector3d center,
			double T, double dt)
		{
			Trajectory<Eigen::Vector3d>* tr = new Trajectory<Eigen::Vector3d>(dt);
			int N = (int)round(T / dt)+1;
			
			double ang = 0.0;
			for (int i = 0; i < N; i++, ang+=toAngle/N)
			{
				Eigen::Vector3d p = start-center;
				Eigen::Vector3d p1(p.x()*cos(ang)-p.y()*sin(ang), 
					p.y()*cos(ang)+p.x()*sin(ang), p.z());
				Eigen::Vector3d p2 = p1+center;
				tr->addPoint(p2);
			}
			return tr;
		}

};

template <>
Trajectory<Pose>* TrajectoryFactory::linearTrajectory<Pose>(
			Pose start,Pose end, double T, double dt)
{
	Trajectory<Pose>* tr = new Trajectory<Pose>(dt);
	int N = (int)round(T / dt)+1;
	for (int i = 0; i < N; i++)
	{
		Pose p = start.interpolate(((double)i)/N, end);
		tr->addPoint(p);
	}
	tr->addPoint(end);
	return tr;
}



#endif
