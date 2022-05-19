/*
 *  utils.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-11-08
 *
 *  Useful functions to provide compatibility
 *  of basic datatypes, mainly in matematical
 *  calculations.
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

#include <std_msgs/Float32.h>
#include <irob_msgs/FloatArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <irob_msgs/ToolPose.h>
#include <irob_msgs/ToolPoseStamped.h>
#include <irob_msgs/Environment.h>

namespace saf {

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


/*std::ostream& operator<<(std::ostream& out, const Eigen::Affine3d& T) {
  out << "Translation:\t" << T.translation() << "\tRotation:\t" << T.rotation();
  return out;
}*/

// Interpolation

template<typename T>
inline T interpolate(double a, T const& x1, T const& x2) {
  return ((1.0-a) * x1) + ((a) * x2);
}


template <>
inline Eigen::Quaterniond interpolate(double a,
                                             const Eigen::Quaterniond& x1,
                                             const Eigen::Quaterniond& x2) {
  return x1.slerp(a, x2);
}

// Distance

template<typename T>
inline double distanceEuler(T const& x1, T const& x2) {
  return std::abs(x2 - x1);
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
inline irob_msgs::Environment unwrapMsg(const irob_msgs::Environment& msg){
  return msg;
}


template <>
inline double unwrapMsg(const std_msgs::Float32& msg){
  return msg.data;
}


template <>
inline Eigen::Vector3d unwrapMsg(const geometry_msgs::Vector3& msg){
  Eigen::Vector3d ret(msg.x, msg.y, msg.z);
  return ret;
}

template <>
inline Eigen::Vector3d unwrapMsg(const geometry_msgs::Point& msg){
  Eigen::Vector3d ret(msg.x, msg.y, msg.z);
  return ret;
}



template <>
inline Eigen::Quaterniond unwrapMsg(const geometry_msgs::Quaternion& msg){
  Eigen::Quaterniond ret(msg.w, msg.x, msg.y, msg.z);
  return ret;
}

template <>
inline std::vector<double> unwrapMsg(const irob_msgs::FloatArray& msg){
  return msg.data;
}

template <>
inline Eigen::Affine3d unwrapMsg(const geometry_msgs::Transform& msg){
  Eigen::Quaterniond q(msg.rotation.w, msg.rotation.x,
                            msg.rotation.y,msg.rotation.z);
  Eigen::Translation3d t(msg.translation.x, msg.translation.y, msg.translation.z);
  Eigen::Affine3d ret(t * q);
  return ret;
}

template <>
inline Eigen::Affine3d unwrapMsg(const geometry_msgs::Pose& msg){
  Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                            msg.orientation.y,msg.orientation.z);
  Eigen::Translation3d t(msg.position.x, msg.position.y, msg.position.z);
  Eigen::Affine3d ret(t * q);
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
inline sensor_msgs::JointState wrapToMsg(const double& data){
  sensor_msgs::JointState msg;
  msg.name.push_back("jaw");
  msg.position.push_back(data);
  return msg;
}


template <>
inline geometry_msgs::Vector3 wrapToMsg(const Eigen::Vector3d& data){
  geometry_msgs::Vector3 msg;
  msg.x = data.x();
  msg.y = data.y();
  msg.z = data.z();
  return msg;
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
inline geometry_msgs::Quaternion wrapToMsg(
    const Eigen::Quaterniond& data){
  geometry_msgs::Quaternion msg;
  msg.w = data.w();
  msg.x = data.x();
  msg.y = data.y();
  msg.z = data.z();
  return msg;
}

template <>
inline geometry_msgs::Transform wrapToMsg(
    const Eigen::Affine3d& data){
  geometry_msgs::Transform msg;

  Eigen::Vector3d translation(data.translation());
  Eigen::Quaterniond rotation(data.rotation());
  msg.translation.x = translation.x();
  msg.translation.y = translation.y();
  msg.translation.z = translation.z();

  msg.rotation.x = rotation.x();
  msg.rotation.y = rotation.y();
  msg.rotation.z = rotation.z();
  msg.rotation.w = rotation.w();
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
inline Eigen::Vector3d makeNaN(){
  Eigen::Vector3d ret(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());
  return ret;
}


template <>
inline Eigen::Quaterniond makeNaN(){
  Eigen::Quaterniond ret(std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN());
  return ret;
}

template <>
inline Eigen::Affine3d makeNaN(){
  Eigen::Affine3d ret(Eigen::Translation3d(makeNaN<Eigen::Vector3d>()));
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
  geometry_msgs::Pose nanp;
  nanp.position.x = std::numeric_limits<double>::quiet_NaN();
  nanp.position.y = std::numeric_limits<double>::quiet_NaN();
  nanp.position.z = std::numeric_limits<double>::quiet_NaN();
  nanp.orientation.x = std::numeric_limits<double>::quiet_NaN();
  nanp.orientation.y = std::numeric_limits<double>::quiet_NaN();
  nanp.orientation.z = std::numeric_limits<double>::quiet_NaN();
  nanp.orientation.w = std::numeric_limits<double>::quiet_NaN();
  return nanp;
}

template <>
inline irob_msgs::ToolPose makeNaN(){
  irob_msgs::ToolPose nanp;
  nanp.transform.translation.x = std::numeric_limits<double>::quiet_NaN();
  nanp.transform.translation.y = std::numeric_limits<double>::quiet_NaN();
  nanp.transform.translation.z = std::numeric_limits<double>::quiet_NaN();
  nanp.transform.rotation.x = std::numeric_limits<double>::quiet_NaN();
  nanp.transform.rotation.y = std::numeric_limits<double>::quiet_NaN();
  nanp.transform.rotation.z = std::numeric_limits<double>::quiet_NaN();
  nanp.transform.rotation.w = std::numeric_limits<double>::quiet_NaN();
  nanp.jaw = std::numeric_limits<double>::quiet_NaN();
  return nanp;
}

template <>
inline irob_msgs::Environment makeNaN(){
  irob_msgs::Environment nanp;
  nanp.valid = irob_msgs::Environment::INVALID;
  return nanp;
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
inline geometry_msgs::Transform makeNaN(){
  geometry_msgs::Transform msg;
  msg.translation.x = std::numeric_limits<double>::quiet_NaN();
  msg.translation.y = std::numeric_limits<double>::quiet_NaN();
  msg.translation.z = std::numeric_limits<double>::quiet_NaN();

  msg.rotation.x = std::numeric_limits<double>::quiet_NaN();
  msg.rotation.y = std::numeric_limits<double>::quiet_NaN();
  msg.rotation.z = std::numeric_limits<double>::quiet_NaN();
  msg.rotation.w = std::numeric_limits<double>::quiet_NaN();
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
inline bool isnan(const Eigen::Vector3d& d)
{
  return (std::isnan(d.x())
          || std::isnan(d.y())
          || std::isnan(d.z()));
}


template <>
inline bool isnan(const Eigen::Quaterniond& d)
{
  return (std::isnan(d.x())
          || std::isnan(d.y())
          || std::isnan(d.z())
          || std::isnan(d.w()));
}

template <>
inline bool isnan(const Eigen::Affine3d& d)
{
  return (d.translation().hasNaN() || d.rotation().hasNaN());
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
  return (std::isnan(d.transform.translation.x)
          || std::isnan(d.transform.translation.y)
          || std::isnan(d.transform.translation.z)
          || std::isnan(d.transform.rotation.x)
          || std::isnan(d.transform.rotation.y)
          || std::isnan(d.transform.rotation.z)
          || std::isnan(d.transform.rotation.w)
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
inline bool isnan(const geometry_msgs::Transform& d)
{
  return (std::isnan(d.translation.x)
          || std::isnan(d.translation.y)
          || std::isnan(d.translation.z)
          || std::isnan(d.rotation.x)
          || std::isnan(d.rotation.y)
          || std::isnan(d.rotation.z)
          || std::isnan(d.rotation.w));
}


template <>
inline bool isnan(const geometry_msgs::Quaternion& d)
{
  return (std::isnan(d.x)
          || std::isnan(d.y)
          || std::isnan(d.z)
          || std::isnan(d.w));
}

template <>
inline bool isnan(const irob_msgs::Environment& d)
{
  return (d.valid == irob_msgs::Environment::INVALID);

}

// Unit vector + rotation to quat
template<typename QuatT, typename VecT>
inline QuatT vecToQuat(const VecT& vec, double angle);

template <>
inline Eigen::Quaterniond vecToQuat(const Eigen::Vector3d& vec,
                                           double angle){
  Eigen::Quaterniond quat_start(0.0, 0.707107, 0.707106, 0.0);
  double angle_rad = (angle / 180.0) * M_PI;
  Eigen::Matrix3d R1m;
  R1m = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond R1(R1m);

  Eigen::Quaterniond ret = R1 * quat_start;

  Eigen::Vector3d vec_start(0.0, 0.0, -1.0);
  Eigen::Quaterniond R2 =
      Eigen::Quaterniond::FromTwoVectors(vec_start, vec);
  ret = R2 * ret;
  return ret;
}

// Quat to unit vector
template<typename QuatT, typename VecT>
inline VecT quatToVec(const QuatT& quat);

template <>
inline Eigen::Vector3d quatToVec(const Eigen::Quaterniond& quat){

  Eigen::Quaterniond quat_start(0.0, 0.707107, 0.707106, 0.0);

  Eigen::Quaterniond R = quat * quat_start.inverse();

  Eigen::Vector3d vec_start(0.0, 0.0, -1.0);
  Eigen::Vector3d ret = R * vec_start;
  return ret;
}

















}

#endif /* DVRK_UTILS_HPP_ */





















