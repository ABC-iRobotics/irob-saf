/*
 *  pose.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-27
 *  
 */

#ifndef DVRK_POSE_HPP_
#define DVRK_POSE_HPP_

#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>

namespace dvrk {

class Pose {
 
   public:
   	
   	struct Distance { 
    	double cartesian;
    	double angle;
    	double jaw;
    	
    	Distance operator*=(double);
    	Distance operator/=(double);
    	Distance operator*(double) const;
    	Distance operator/(double) const;
    	
    	friend std::ostream& operator<<(std::ostream&, const Distance&);
  	};
   
   	Eigen::Vector3d position;
   	Eigen::Quaternion<double> orientation;
   	double jaw;
   	
   	Pose();
   	Pose(double, double, double, double, double, double, double, double);
   	Pose(const Pose&);
	Pose(const geometry_msgs::Pose&, double jaw);
	Pose(const geometry_msgs::PoseStamped&, double jaw);
   	Pose(const Eigen::Vector3d&, const Eigen::Quaternion<double>&, double);
   	
   	void swap(Pose&);
   	Pose operator=(const Pose&);
   	
   	Pose operator+=(const Eigen::Vector3d&);
   	Pose operator-=(const Eigen::Vector3d&);
   	
   	Pose operator+(const Eigen::Vector3d&) const;
   	Pose operator-(const Eigen::Vector3d&) const;
   	
   	Pose interpolate(double, const Pose&) const;
   	
   	bool isNaN() const;
   	
   	Distance dist(const Pose&) const;
   	Distance dist(const Eigen::Vector3d&) const;
   	Distance dist(const Eigen::Quaternion<double>&) const;
   	Distance dist(double) const;
   	
   	geometry_msgs::Pose toRosPose() const;
    std_msgs::Float32 toRosJaw() const;
   	
   	/*
   	double length() const;
   	
   	double distance(const Eigen::Vector3d&) const;
   */
   	friend std::ostream& operator<<(std::ostream&, const Pose&);
	friend std::istream& operator>>(std::istream&, Pose&);
};
}
#endif
