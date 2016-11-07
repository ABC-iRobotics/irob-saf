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
#include <math.h>

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
			for (int i = 0; i < N-1; i++)
			{
				P p = ((start*((N-i)/(double)N)) +(end*(i/(double)N)));
				tr->addPoint(p);
			}
			tr->addPoint(end);
			return tr;
		}
		
		static Trajectory<Vector3D>* circleTrajectoryHorizontal(Vector3D start, 
			double toAngle, Vector3D center,
			double T, double dt)
		{
			Trajectory<Vector3D>* tr = new Trajectory<Vector3D>(dt);
			int N = (int)round(T / dt)+1;
			
			double ang = 0.0;
			for (int i = 0; i < N; i++, ang+=toAngle/N)
			{
				Vector3D p = start-center;
				Vector3D p1(p.x*cos(ang)-p.y*sin(ang), 
					p.y*cos(ang)+p.x*sin(ang), p.z);
				Vector3D p2 = p1+center;
				tr->addPoint(p2);
			}
			return tr;
		}

};



#endif
