/*
 * trajectory.hpp
 *
 *  Created on: 2016. okt. 24.
 *      Author: tamas
 */

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <string>
#include <stdexcept>


template <class T>
class Trajectory 
{
 	public:
 		double dt;
 	private:
 		std::vector<T> points;
   	public:
   		Trajectory(double);
   		Trajectory(std::string);
		void clear();
		void addPoint(T);
		T operator[](const int) const;
		int size() const;
		double getLengthSec() const;
		void writeToFile(std::string) const;
	
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
Trajectory<T>::Trajectory(std::string filename)
{
	std::ifstream logfile(filename.c_str());
    if (!logfile.is_open())
    	throw std::runtime_error("Cannot open file " + filename);
    if (logfile.eof())
    	throw std::runtime_error("Logfile " + filename + " is empty.");
   	// The first line contains the dt
    logfile >> dt;
    
    while (!logfile.eof())
    {
    		T v;
      		logfile >> v;
      		points.push_back(v);
      		//std::cout << v << std::endl;
    }
    logfile.close();
}

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

template <class T>
void Trajectory<T>::writeToFile(std::string filename) const
{
	std::ofstream logfile;
    logfile.open (filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    
    if (!logfile.is_open())
		throw std::runtime_error("Cannot open file " + filename);
	std::cout <<"Start logging to  " << filename << std::endl;
	
	// The first line is always the dt
	logfile << dt << std::endl;
	// One line contains one point in the trajectory
	for (int i = 0; i < points.size(); i++)
		logfile << points[i] << std::endl;
	
	logfile.flush();
	logfile.close();
	std::cout << "Trajectory of " << points.size() 
		<<" points succesfully logged to " << filename << std::endl;
}



#endif
