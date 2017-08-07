/*
 *  utils.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-08
 *  
 */

#ifndef DVRK_UTILS_HPP_
#define DVRK_UTILS_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "irob_utils/pose.hpp"

namespace ias {

typedef enum InterpolationMethod 
    	{LINEAR, BEZIER} InterpolationMethod;  

inline double degToRad(double deg) {
   return (deg / 180.0) * M_PI;
}

inline double radToDeg(double rad) {
   return (rad * 180.0) / M_PI;
}

template<typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last) 
            out << ", ";
    }
    out << "]";
    return out;
}

// Interpolation
template<typename T>
inline T interpolate(double a, T const& x1, T const& x2) {
   return ((1.0-a) * x1) + ((a) * x2);
}

template <>
inline Pose interpolate(double a, const Pose& x1, const Pose& x2) {
   return x1.interpolate(a, x2);
}

template <>
inline Eigen::Quaternion<double> interpolate(double a,
									const Eigen::Quaternion<double>& x1,
									const Eigen::Quaternion<double>& x2) {
   return x1.slerp(a, x2);
}

// Distance
template<typename T>
inline double distanceEuler(T const& x1, T const& x2) {
   return std::abs(x2 - x1);
}


template <>
inline double distanceEuler(const Pose& x1, const Pose& x2) {
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
inline double distanceEuler(const Eigen::Vector3d& x1,
									const Eigen::Vector3d& x2) {
   return std::abs((x2-x1).norm());
}



}

#endif /* DVRK_UTILS_HPP_ */
