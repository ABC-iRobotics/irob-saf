/*
 * trajectory_factory.hpp
 *
 *  Created on: 2016. okt. 25.
 *      Author: tamas
 */

#ifndef DVRK_TRAJECTORY_FACTORY_HPP_
#define DVRK_TRAJECTORY_FACTORY_HPP_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "dvrk/trajectory.hpp"
#include "dvrk/pose.hpp"
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

namespace dvrk {

class TrajectoryFactory
{

	private:
		TrajectoryFactory() {}
	public:
		template <class P>
		static Trajectory<P>* linearTrajectoryWithT(P start,P end, 
											double T, double dt)
		{
			Trajectory<P>* tr = new Trajectory<P>(dt);
			int N = (int)round(T / dt);
			for (int i = 0; i <= N; i++)
			{
				P p = ((start*((N-i)/(double)N)) +(end*(i/(double)N)));
				tr->addPoint(p);
			}
			return tr;
		}
		
		template <class P>
		static Trajectory<P>* linearTrajectoryWithSpeed(P start,P end, 
											double speed, double dt)
		{
			Trajectory<P>* tr = new Trajectory<P>(dt);
			double stepSize = speed * dt;
			int N = (int)round(std::abs(end - start) / stepSize);
			for (int i = 0; i <= N; i++)
			{
				P p = ((start*((N-i)/(double)N)) +(end*(i/(double)N)));
				tr->addPoint(p);
			}
			return tr;
		}
		
		static Trajectory<double>* linearTrajectoryWithAcc(double start,
											double end, 
											double speed, double Tacc, double dt)
		{
			Trajectory<double>* tr = new Trajectory<double>(dt);
			if (end < start)
				speed *= -1.0;
				
			double fullS = std::abs(end - start);
			double accS = (speed / 2.0) * Tacc;
			while (fullS < (2.0 * accS))
			{
				Tacc /= 2.0;
				speed /= 2.0;
				accS = (speed / 2) * Tacc;
				ROS_WARN_STREAM(
				"Trajectory too short, speed and acceleration time reduced.");
			}
				
			double posStep = speed * dt;
			double acc = speed / Tacc;
			double speedStep = acc * dt;
			int Nacc = (int)round(speed / speedStep);
			
			int N = (int)round((end - start-(2*accS)) / posStep);
			
			double prevp = start;
			for (int i = 0; i <= Nacc; i++)
			{
				double p = prevp + (i * speedStep * dt);
				tr->addPoint(p);
				prevp = p;
			}
			
			
			for (int i = 0; i <= N; i++)
			{
				double p = prevp + posStep;
				tr->addPoint(p);
				prevp = p;
			}
			
			for (int i = 0; i <= Nacc; i++)
			{
				double p = prevp + ((Nacc - i) * speedStep * dt);
				tr->addPoint(p);
				prevp = p;
			}
			return tr;
		}
		
		static Trajectory<Pose>* linearTrajectoryWithAcc(Pose start,
											Pose end, 
											double speed, double Tacc, double dt)
		{
			Trajectory<Pose>* tr = new Trajectory<Pose>(dt);
			
			double fullS = (end.position - start.position).norm();
			double accS = (speed / 2.0) * Tacc;
			while (fullS < (2.0 * accS))
			{
				Tacc /= 2.0;
				speed /= 2.0;
				accS = (speed / 2) * Tacc;
				ROS_WARN_STREAM(
				"Trajectory too short, speed and acceleration time reduced.");
			}
			
			
			Eigen::Vector3d posStep = speed * dt 
							* (end.position - start.position).normalized();
			double acc = speed / Tacc;
			Eigen::Vector3d speedStep = acc * dt 
							* (end.position - start.position).normalized();
			
			
			Eigen::Vector3d speedVec = speed 
							* (end.position - start.position).normalized();
						
			Pose accEndPose = start.interpolate(accS/fullS, end);
			Pose deccStartPose = start.interpolate(1.0 - (accS/fullS), end);
			int Nacc = (int)round(speed / speedStep.norm());
			int Macc = (Nacc*(Nacc+1))/2;
			int N = (int)round((fullS-(2.0*accS))/ posStep.norm());
			
			for (int i = 0, j = 0; i <= Nacc; j+=(++i))
			{
				Pose p = start.interpolate(((double)j)/Macc
														, accEndPose);
				tr->addPoint(p);
			}
			
			
			for (int i = 0; i <= N; i++)
			{
				Pose p = accEndPose.interpolate(((double)i)/N
														, deccStartPose);
				tr->addPoint(p);
			}
			
			for (int i = Nacc, j = 0; i >= 0; j+=(--i))
			{
				Pose p = deccStartPose.interpolate(((double)j)/Macc
														, end);
				tr->addPoint(p);
			}
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
Trajectory<Pose>* TrajectoryFactory::linearTrajectoryWithT<Pose>(
			Pose start,Pose end, double T, double dt)
{
	Trajectory<Pose>* tr = new Trajectory<Pose>(dt);
	int N = (int)round(T / dt);
	for (int i = 0; i <= N; i++)
	{
		Pose p = start.interpolate(((double)i)/N, end);
		tr->addPoint(p);
	}
	return tr;
}

template <>
Trajectory<Pose>* TrajectoryFactory::linearTrajectoryWithSpeed<Pose>(
			Pose start,Pose end, double speed, double dt)
{
	Trajectory<Pose>* tr = new Trajectory<Pose>(dt);
	double stepSize = speed * dt;
	int N = (int)round((end.position - start.position).norm() / stepSize);
	for (int i = 0; i <= N; i++)
	{
		Pose p = start.interpolate(((double)i)/N, end);
		tr->addPoint(p);
	}
	return tr;
}

template <>
Trajectory<Eigen::Vector3d>* TrajectoryFactory::linearTrajectoryWithSpeed<Eigen::Vector3d>(
			Eigen::Vector3d start,Eigen::Vector3d end, double speed, double dt)
{
	Trajectory<Eigen::Vector3d>* tr = new Trajectory<Eigen::Vector3d>(dt);
	double stepSize = speed * dt;
	int N = (int)round((end - start).norm() / stepSize);
	for (int i = 0; i <= N; i++)
	{
		Eigen::Vector3d p = ((start*((N-i)/(double)N)) +(end*(i/(double)N)));
		tr->addPoint(p);
	}
	tr->addPoint(end);
	return tr;
}


}

#endif
