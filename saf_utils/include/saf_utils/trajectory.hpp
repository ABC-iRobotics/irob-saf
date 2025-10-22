/*
 *  trajectory.hpp
 *
 *  Author(s): Tamas Levendovics
 *  Created on: 2016-10-24
 *
 *  Generic class to store and manipulate trajectories
 *  of any corresponding data type.
 *
 */

#ifndef DVRK_TRAJECTORY_HPP_
#define DVRK_TRAJECTORY_HPP_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <fstream>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <saf_utils/utils.hpp>
#include <saf_utils/tool_pose.hpp>
#include <saf_msgs/msg/trajectory_tool_pose.hpp>  // ROS 2 message include

namespace saf {

template <class T>
class Trajectory
{
public:
  double dt;

private:
  std::vector<T> points;

public:
  Trajectory();
  Trajectory(double);
  Trajectory(std::string);
  Trajectory(const Trajectory<T>&);
  Trajectory(Trajectory<T>&&);
  Trajectory(const saf_msgs::msg::TrajectoryToolPose&); // ROS 2 message
  void clear();
  void addPoint(T);
  T operator[](const int) const;
  void reverse();
  int size() const;
  double getLengthSec() const;
  void writeToFile(std::string) const;

  void swap(Trajectory<T>&);
  Trajectory<T> operator=(const Trajectory<T>&);

  Trajectory<T> operator+=(const Trajectory<T>&);
  Trajectory<T> operator+(const Trajectory<T>&);

  void transform(const Eigen::Affine3d&);

  void copyToRosTrajectory(saf_msgs::msg::TrajectoryToolPose&); // ROS 2 message

  friend std::ostream& operator<<(std::ostream& os, const Trajectory<T>& tr)
  {
    // Output trajectory to stream
    os << "Trajectory:" << std::endl;
    for (int i = 0; i < tr.size(); i++)
      os << tr[i] << std::endl;
    return os;
  }
};

// Implementations

template <class T>
Trajectory<T>::Trajectory() : dt(0.1) {}

template <class T>
Trajectory<T>::Trajectory(double dt) : dt(dt) {}

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
  }
  logfile.close();
}

template <class T>
Trajectory<T>::Trajectory(const Trajectory<T>& other) : dt(other.dt)
{
  for (int i = 0; i < other.size(); i++)
    points.push_back(other[i]);
}

template <class T>
Trajectory<T>::Trajectory(Trajectory<T>&& other) : dt(other.dt), points(std::move(other.points)) {}

template <class T>
Trajectory<T>::Trajectory(const saf_msgs::msg::TrajectoryToolPose& msg)
{
  // Convert from ROS 2 message to trajectory
  // This depends on your message structure and how you convert it to T
}

template <class T>
void Trajectory<T>::swap(Trajectory<T>& other)
{
  Trajectory<T> tmp(*this);
  dt = other.dt;
  points = other.points;
  other.dt = tmp.dt;
  other.points = tmp.points;
}

template <class T>
Trajectory<T> Trajectory<T>::operator=(const Trajectory<T>& other)
{
  Trajectory<T> tmp(other);
  this->swap(tmp);
  return *this;
}

template <class T>
Trajectory<T> Trajectory<T>::operator+=(const Trajectory<T>& other)
{
  if (dt != other.dt)
    // Handle error case
    for (T p : other.points)
      points.push_back(p);

  return *this;
}

template <class T>
Trajectory<T> Trajectory<T>::operator+(const Trajectory<T>& other)
{
  Trajectory<T> tmp(*this);
  tmp += other;
  return tmp;
}

template <class T>
void Trajectory<T>::transform(const Eigen::Affine3d& G)
{
  for (T& p : points)
    p = G * p;
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
void Trajectory<T>::reverse()
{
  std::reverse(points.begin(), points.end());
}

template <class T>
int Trajectory<T>::size() const
{
  return points.size();
}

template <class T>
double Trajectory<T>::getLengthSec() const
{
  return (points.size() - 1) * dt;
}

template <class T>
void Trajectory<T>::writeToFile(std::string filename) const
{
  std::ofstream logfile;
  logfile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

  if (!logfile.is_open())
    throw std::runtime_error("Cannot open file " + filename);

  std::cout << "Start logging to " << filename << std::endl;

  // The first line is always the dt
  logfile << dt << std::endl;
  // One line contains one point in the trajectory
  for (T p : points)
    logfile << p << std::endl;

  logfile.flush();
  logfile.close();
  std::cout << "Trajectory of " << points.size() << " points successfully logged to " << filename << std::endl;
}

} // namespace saf

#endif
