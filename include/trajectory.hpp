/*
 * trajectory.hpp
 *
 *  Created on: 2016. okt. 24.
 *      Author: tamas
 */

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <iostream>
#include <vector>

template <class T>
class Trajectory 
{
 	public:
 		double dt;
 	private:
 		std::vector<T> points;
   	public:
   		Trajectory(double);
		void clear();
		void addPoint(T);
		T operator[](const int) const;
		int size() const;
		double getLengthSec() const;
	
		friend std::ostream& operator<<(std::ostream& os, const Trajectory<T>& tr)
		{		
			//TODO fix
			os << "Trajectory:" << std::endl;
			for (int i = 0; i < tr.size(); i++)
				os << tr[i] << std::endl;
   			return os;
		}
   
};


template <class T>
Trajectory<T>::Trajectory(double dt): dt(dt){}

template <class T>
void Trajectory<T>::clear()
{
	points.clear();
}

template <class T>
void Trajectory<T>::addPoint(T p)
{
	points.push_back(p);
}
		
template <class T>
T Trajectory<T>::operator[](const int i) const
{
	return points[i];
}
		
template <class T>
int Trajectory<T>::size() const
{
	return points.size();
}
		
template <class T>
double Trajectory<T>::getLengthSec() const
{
	return (points.size()-1)*dt;
}



#endif
