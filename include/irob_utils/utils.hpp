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
#include <limits>
#include "irob_utils/pose.hpp"

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <irob_autosurg/ToolPose.h>
#include <irob_autosurg/ToolPoseStamped.h>

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

// Conversion from ROS msg
template<typename MsgT, typename DataT>
inline DataT unwrapMsg(const MsgT& msg);


template <>
inline double unwrapMsg(const std_msgs::Float32& msg){
	return msg.data;
}

template <>
inline Pose unwrapMsg(const geometry_msgs::Pose& msg){
	Pose ret(msg, 0);
	return ret;
}

template <>
inline Pose unwrapMsg(const irob_autosurg::ToolPose& msg){
	Pose ret(msg);
	return ret;
}

template <>
inline Pose unwrapMsg(const geometry_msgs::PoseStamped& msg){
	Pose ret(msg, 0);
	return ret;
}

template <>
inline Pose unwrapMsg(const irob_autosurg::ToolPoseStamped& msg){
	Pose ret(msg);
	return ret;
}

template <>
inline Eigen::Vector3d unwrapMsg(const geometry_msgs::Point& msg){
	Eigen::Vector3d ret(msg.x, msg.y, msg.z);
	return ret;
}

template <>
inline Eigen::Vector3d unwrapMsg(const geometry_msgs::PointStamped& msg){
	Eigen::Vector3d ret(msg.point.x, msg.point.y, msg.point.z);
	return ret;
}

template <>
inline Eigen::Quaternion<double> unwrapMsg(const geometry_msgs::Quaternion& msg){
	Eigen::Quaternion<double> ret(msg.w, msg.x, msg.y, msg.z);
	return ret;
}


// Conversion to ROS msg
template<typename MsgT, typename DataT>
inline MsgT wrapToMsg(const DataT& data);

template <>
inline std_msgs::Float32 wrapToMsg(const double& data){
	std_msgs::Float32 msg;
   	msg.data = data;
	return msg;
}

template <>
inline geometry_msgs::Pose wrapToMsg(const Pose& data){
	return data.toRosPose();
}

template <>
inline irob_autosurg::ToolPose wrapToMsg(const Pose& data){
	return data.toRosToolPose();
}

template <>
inline geometry_msgs::Point wrapToMsg(const Eigen::Vector3d& data){
	geometry_msgs::Point msg;
   	msg.x = data.x();
   	msg.y = data.y();
   	msg.z = data.z();
	return msg;
}

template <>
inline geometry_msgs::PointStamped wrapToMsg(const Eigen::Vector3d& data){
	geometry_msgs::PointStamped msg;
   	msg.point.x = data.x();
   	msg.point.y = data.y();
   	msg.point.z = data.z();
	return msg;
}

template <>
inline geometry_msgs::Quaternion wrapToMsg(
									const Eigen::Quaternion<double>& data){
	geometry_msgs::Quaternion msg;
	msg.w = data.w();
   	msg.x = data.x();
   	msg.y = data.y();
   	msg.z = data.z();
	return msg;
}

// NaN
template<typename DataT>
inline DataT makeNaN();


template <>
inline double makeNaN(){
	return std::numeric_limits<double>::quiet_NaN();
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
inline Eigen::Vector3d makeNaN(){
	Eigen::Vector3d ret(std::numeric_limits<double>::quiet_NaN(),
						std::numeric_limits<double>::quiet_NaN(),
						std::numeric_limits<double>::quiet_NaN());
	return ret;
}


template <>
inline Eigen::Quaternion<double> makeNaN(){
	Eigen::Quaternion<double> ret(std::numeric_limits<double>::quiet_NaN(),
								std::numeric_limits<double>::quiet_NaN(),
								std::numeric_limits<double>::quiet_NaN(),
								std::numeric_limits<double>::quiet_NaN());
	return ret;
}

template <>
inline std_msgs::Float32 makeNaN(){
	std_msgs::Float32 msg;
   	msg.data = std::numeric_limits<double>::quiet_NaN();
	return msg;
}

template <>
inline geometry_msgs::Pose makeNaN(){
	Pose nanp = makeNaN<Pose>();
	return nanp.toRosPose();
}

template <>
inline irob_autosurg::ToolPose makeNaN(){
	Pose nanp = makeNaN<Pose>();
	return nanp.toRosToolPose();
}

template <>
inline geometry_msgs::Point makeNaN(){
	geometry_msgs::Point msg;
   	msg.x = std::numeric_limits<double>::quiet_NaN();
   	msg.y = std::numeric_limits<double>::quiet_NaN();
   	msg.z = std::numeric_limits<double>::quiet_NaN();
	return msg;
}

template <>
inline geometry_msgs::PointStamped makeNaN(){
	geometry_msgs::PointStamped msg;
   	msg.point.x = std::numeric_limits<double>::quiet_NaN();
   	msg.point.y = std::numeric_limits<double>::quiet_NaN();
   	msg.point.z = std::numeric_limits<double>::quiet_NaN();
	return msg;
}

template <>
inline geometry_msgs::Quaternion makeNaN(){
	geometry_msgs::Quaternion msg;
	msg.w = std::numeric_limits<double>::quiet_NaN();
   	msg.x = std::numeric_limits<double>::quiet_NaN();
   	msg.y = std::numeric_limits<double>::quiet_NaN();
   	msg.z = std::numeric_limits<double>::quiet_NaN();
	return msg;
}



}

#endif /* DVRK_UTILS_HPP_ */
