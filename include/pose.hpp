/*
 * vector_3dpose
 *  Created on: 2016. okt. 27.
 *      Author: tamas
 */

#ifndef POSE_HPP_
#define POSE_HPP_

#include <iostream>
#include "vector_3d.hpp"
#include "quaternion.hpp"
#include "geometry_msgs/PoseStamped.h"

class Pose {
 
   public:
   	Vector3D position;
   	Quaternion orientation;
   	double jaw;
   	
   	Pose();
   	Pose(double, double, double, double, double, double, double, double);
   	Pose(const Pose&);
	Pose(const geometry_msgs::PoseStamped&, double jaw);
   	Pose(const Vector3D&, const Quaternion&, double);
   	
   	void swap(Pose&);
   	Pose operator=(const Pose&);
   	
   	Pose operator+=(const Vector3D&);
   	Pose operator-=(const Vector3D&);
   	
   	Pose operator+=(const Quaternion&);
   	Pose operator-=(const Quaternion&);
   	
   	Pose operator+=(const Pose&);
   	Pose operator-=(const Pose&);
   	
   	// jaw
   	Pose operator+=(const double&);
   	Pose operator-=(const double&);
   	
   	Pose operator*=(const double&);
   	Pose operator/=(const double&);
   	
   	Pose operator+(const Vector3D&) const;
   	Pose operator-(const Vector3D&) const;
   	
   	Pose operator+(const Quaternion&) const;
   	Pose operator-(const Quaternion&) const;
   	
   	Pose operator+(const Pose&) const;
   	Pose operator-(const Pose&) const;
   	
   	//jaw
   	Pose operator+(const double&) const;
   	Pose operator-(const double&) const;
   	
   	Pose operator*(const double&) const;
   	Pose operator/(const double&) const;
   	
   	/*
   	double length() const;
   	
   	double distance(const Vector3D&) const;
   */
   	friend std::ostream& operator<<(std::ostream&, const Pose&);
	friend std::istream& operator>>(std::istream&, Pose&);
};

#endif
