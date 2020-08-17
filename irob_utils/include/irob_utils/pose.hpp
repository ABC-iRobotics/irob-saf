/*
 *  pose.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-27
 *
 *  Class to store and make calculations on a robot arm,
 *  including the angle of the grippers.
 *  
 */

#ifndef IROB_POSE_HPP_
#define IROB_POSE_HPP_

#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include <irob_msgs/ToolPose.h>
#include <irob_msgs/ToolPoseStamped.h>
#include <ros/ros.h>
#include <irob_utils/utils.hpp>




namespace saf {

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
   	Pose(const irob_msgs::ToolPose&);
   	Pose(const irob_msgs::ToolPoseStamped&);
    Pose(const geometry_msgs::Pose&, double jaw);
    Pose(const geometry_msgs::PoseStamped&, double jaw);
   	Pose(const Eigen::Vector3d&, const Eigen::Quaternion<double>&, double);
    Pose(const geometry_msgs::Point&, const Eigen::Quaternion<double>&, double);
    Pose(const Eigen::Transform<double,3,Eigen::Affine>&, double );

   	void swap(Pose&);
   	Pose operator=(const Pose&);
   	
   	Pose operator+=(const Eigen::Vector3d&);
   	Pose operator-=(const Eigen::Vector3d&);
   	
   	Pose operator+(const Eigen::Vector3d&) const;
   	Pose operator-(const Eigen::Vector3d&) const;
   	
   	
   	Pose interpolate(double, const Pose&) const;
   	
   	Pose rotate(const Eigen::Matrix3d&) const;
    Pose transform(const Eigen::Transform<double,3,Eigen::Affine>&);
    Pose invTransform(const Eigen::Transform<double,3,Eigen::Affine>&);
    Pose transform(const Eigen::Matrix3d&, const Eigen::Vector3d&, double = 1.0);
   	Pose invTransform(const Eigen::Matrix3d&, const Eigen::Vector3d&, double = 1.0);
   	
    Pose transform(const geometry_msgs::Transform&, double = 1.0);
    Pose invTransform(const geometry_msgs::Transform&, double = 1.0);

   	bool isNaN() const;
   	
   	Distance dist(const Pose&) const;
   	Distance dist(const Eigen::Vector3d&) const;
   	Distance dist(const Eigen::Quaternion<double>&) const;
   	Distance dist(double) const;
   	
    irob_msgs::ToolPose toRosToolPose() const;
   	geometry_msgs::Pose toRosPose() const;
    Eigen::Transform<double,3,Eigen::Affine> toTransform() const;
    sensor_msgs::JointState toRosJaw() const;
   	
   	friend std::ostream& operator<<(std::ostream&, const Pose&);
    friend std::istream& operator>>(std::istream&, Pose&);
    friend Pose operator*(const Eigen::Matrix3d&, const Pose&);
    friend Pose operator*(const Eigen::Transform<double,3,Eigen::Affine>&, const Pose&);
};

// Template specializations from utils.hpp

template<> inline Pose interpolate(double a, const Pose& x1, const Pose& x2) {
  return x1.interpolate(a, x2);
}

template<> inline double distanceEuler(const Pose& x1, const Pose& x2) {
  Pose::Distance d = x1.dist(x2);
  double weighted_cartesian = std::abs(d.cartesian) * 10000.0;
  double weighted_angle = std::abs(d.angle);
  double weighted_jaw = radToDeg(std::abs(d.jaw));
  if (weighted_cartesian >= weighted_angle
      && weighted_cartesian >= weighted_jaw)
    return std::abs(d.cartesian);
  if (weighted_angle >= weighted_jaw)
    // in degrees, should be converted to rad?
    return std::abs(d.angle);
  return std::abs(d.jaw);
}


template <>
inline Pose unwrapMsg(const geometry_msgs::Pose& msg){
  Pose ret(msg, 0);
  return ret;
}

template <>
inline Pose unwrapMsg(const irob_msgs::ToolPose& msg){
  Pose ret(msg);
  return ret;
}

template <>
inline Pose unwrapMsg(const geometry_msgs::PoseStamped& msg){
  Pose ret(msg, 0);
  return ret;
}

template <>
inline Pose unwrapMsg(const irob_msgs::ToolPoseStamped& msg){
  Pose ret(msg);
  return ret;
}

template <>
inline geometry_msgs::Pose wrapToMsg(const Pose& data){
  return data.toRosPose();
}

template <>
inline irob_msgs::ToolPose wrapToMsg(const Pose& data){
  return data.toRosToolPose();
}

template <>
inline Pose makeNaN(){
  Pose ret(std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN(),
           std::numeric_limits<double>::quiet_NaN());
  return ret;
}

template <>
inline bool isnan(const Pose& d)
{
  return (std::isnan(d.position.x())
          || std::isnan(d.position.y())
          || std::isnan(d.position.z())
          || std::isnan(d.orientation.x())
          || std::isnan(d.orientation.y())
          || std::isnan(d.orientation.z())
          || std::isnan(d.orientation.w())
          || std::isnan(d.jaw));
}

}
#endif
