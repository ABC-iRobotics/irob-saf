/*
 * vector_3dpose
 *  Created on: 2016. okt. 27.
 *      Author: tamas
 */

#ifndef POSE_HPP_
#define POSE_HPP_

#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 

class Pose {
 
   public:
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
   	
   	geometry_msgs::Pose toRosPose() const;
    std_msgs::Float32 toRosJaw() const;
   	
   	/*
   	double length() const;
   	
   	double distance(const Eigen::Vector3d&) const;
   */
   	friend std::ostream& operator<<(std::ostream&, const Pose&);
	friend std::istream& operator>>(std::istream&, Pose&);
};

#endif
