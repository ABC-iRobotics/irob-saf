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

#ifndef IROB_TOOL_POSE_HPP_
#define IROB_TOOL_POSE_HPP_

#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
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

class ToolPose {
 
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

   
    Eigen::Affine3d transform;
   	double jaw;
   	
    ToolPose();
    ToolPose(double, double, double, double, double, double, double, double);
    ToolPose(const ToolPose&);
    ToolPose(const irob_msgs::ToolPose&);
    ToolPose(const irob_msgs::ToolPoseStamped&);
    ToolPose(const Eigen::Vector3d&, const Eigen::Quaterniond&, double);
    ToolPose(const Eigen::Affine3d&, double );
    ToolPose(const Eigen::Translation3d&, const Eigen::Quaterniond&, double );
    ToolPose(const geometry_msgs::Transform&, double);
    ToolPose(const geometry_msgs::TransformStamped&, double);
    ToolPose(const geometry_msgs::Pose&, double);
    ToolPose(const geometry_msgs::PoseStamped&, double);

    irob_msgs::ToolPose toRosToolPose() const;

    void swap(ToolPose&);
    ToolPose operator=(const ToolPose&);
   	
    ToolPose interpolate(double, const ToolPose&) const;
   	
   	bool isNaN() const;
   	
    Distance dist(const ToolPose&) const;
    Distance dist(const Eigen::Affine3d&) const;
   	Distance dist(const Eigen::Vector3d&) const;
    Distance dist(const Eigen::Quaterniond&) const;
   	Distance dist(double) const;
   	
   	
    friend std::ostream& operator<<(std::ostream&, const ToolPose&);
    friend std::istream& operator>>(std::istream&, ToolPose&);

    template<typename T_typ>
    friend ToolPose operator*(
          const T_typ& T,
          const ToolPose& p)
    {
      ToolPose ret(T * p.transform, p.jaw);
      return ret;
    }
};

// Template specializations from utils.hpp

template<> inline ToolPose interpolate(double a, const ToolPose& x1, const ToolPose& x2) {
  return x1.interpolate(a, x2);
}

template<> inline double distanceEuler(const ToolPose& x1, const ToolPose& x2) {
  ToolPose::Distance d = x1.dist(x2);
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
inline ToolPose unwrapMsg(const irob_msgs::ToolPose& msg){
  ToolPose ret(msg);
  return ret;
}


template <>
inline ToolPose unwrapMsg(const irob_msgs::ToolPoseStamped& msg){
  ToolPose ret(msg);
  return ret;
}


template <>
inline irob_msgs::ToolPose wrapToMsg(const ToolPose& data){
  return data.toRosToolPose();
}

template <>
inline ToolPose makeNaN(){
  ToolPose ret(std::numeric_limits<double>::quiet_NaN(),
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
inline bool isnan(const ToolPose& d)
{
  Eigen::Translation3d v(d.transform.translation());
  Eigen::Quaterniond q(d.transform.rotation());
  return (std::isnan(v.x())
          || std::isnan(v.y())
          || std::isnan(v.z())
          || std::isnan(q.x())
          || std::isnan(q.y())
          || std::isnan(q.z())
          || std::isnan(q.w())
          || std::isnan(d.jaw));
}

}
#endif
