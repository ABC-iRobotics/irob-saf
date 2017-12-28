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
#include <irob_msgs/FloatArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <irob_msgs/ToolPose.h>
#include <irob_msgs/ToolPoseStamped.h>

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

template <>
inline std::vector<double> unwrapMsg(const irob_msgs::FloatArray& msg){
	return msg.data;
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
inline irob_msgs::ToolPose wrapToMsg(const Pose& data){
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
inline irob_msgs::FloatArray makeNaN(){
	irob_msgs::FloatArray msg;
   	msg.data.push_back(std::numeric_limits<double>::quiet_NaN());
	return msg;
}

template <>
inline geometry_msgs::Pose makeNaN(){
	Pose nanp = makeNaN<Pose>();
	return nanp.toRosPose();
}

template <>
inline irob_msgs::ToolPose makeNaN(){
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


// isnan
template<typename DataT>
inline bool isnan(const DataT& d);


template <>
inline bool isnan(const double& d)
{
	return std::isnan(d);
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


template <>
inline bool isnan(const Eigen::Vector3d& d)
{
	return (std::isnan(d.x())
			|| std::isnan(d.y())
			|| std::isnan(d.z()));
}


template <>
inline bool isnan(const Eigen::Quaternion<double>& d)
{
	return (std::isnan(d.x()) 
			|| std::isnan(d.y())
			|| std::isnan(d.z())
			|| std::isnan(d.w()));
}

template <>
inline bool isnan(const std_msgs::Float32& d)
{
	
	return (std::isnan(d.data));
}

template <>
inline bool isnan(const geometry_msgs::Pose& d)
{
	return (std::isnan(d.position.x) 
			|| std::isnan(d.position.y) 
			|| std::isnan(d.position.z) 
			|| std::isnan(d.orientation.x) 
			|| std::isnan(d.orientation.y) 
			|| std::isnan(d.orientation.z) 
			|| std::isnan(d.orientation.w));
}

template <>
inline bool isnan(const irob_msgs::ToolPose& d)
{
	return (std::isnan(d.position.x) 
			|| std::isnan(d.position.y) 
			|| std::isnan(d.position.z) 
			|| std::isnan(d.orientation.x) 
			|| std::isnan(d.orientation.y) 
			|| std::isnan(d.orientation.z) 
			|| std::isnan(d.orientation.w)
			|| std::isnan(d.jaw));
}

template <>
inline bool isnan(const geometry_msgs::Point& d)
{
	return (std::isnan(d.x) 
			|| std::isnan(d.y) 
			|| std::isnan(d.z) );
}

template <>
inline bool isnan(const geometry_msgs::PointStamped& d)
{
	return isnan<geometry_msgs::Point>(d.point);
}

template <>
inline bool isnan(const geometry_msgs::Quaternion& d)
{
	return (std::isnan(d.x)
			|| std::isnan(d.y)
			|| std::isnan(d.z)
			|| std::isnan(d.w));
}

// Unit vector + rotation to quat
template<typename QuatT, typename VecT>
inline QuatT vecToQuat(const VecT& vec, double angle);

template <>
inline Eigen::Quaternion<double> vecToQuat(const Eigen::Vector3d& vec,
												double angle){
	Eigen::Quaternion<double> quat_start(0.0, 0.707107, 0.707106, 0.0);
	double angle_rad = (angle / 180.0) * M_PI;
	Eigen::Matrix3d R1m;
	R1m = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
  		* Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY())
  		* Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ());
  	Eigen::Quaternion<double> R1(R1m);
  	
  	Eigen::Quaternion<double> ret = R1 * quat_start;
	
	Eigen::Vector3d vec_start(0.0, 0.0, -1.0);
	Eigen::Quaternion<double> R2 = 
		Eigen::Quaternion<double>::FromTwoVectors(vec_start, vec);
	ret = R2 * ret;									
	return ret;
}

// Quat to unit vector
template<typename QuatT, typename VecT>
inline VecT quatToVec(const QuatT& quat);

template <>
inline Eigen::Vector3d quatToVec(const Eigen::Quaternion<double>& quat){

	Eigen::Quaternion<double> quat_start(0.0, 0.707107, 0.707106, 0.0);

	Eigen::Quaternion<double> R = quat * quat_start.inverse();
	
	Eigen::Vector3d vec_start(0.0, 0.0, -1.0);
	Eigen::Vector3d ret = R * vec_start;									
	return ret;
}

















}

#endif /* DVRK_UTILS_HPP_ */





















