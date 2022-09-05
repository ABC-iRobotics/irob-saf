/*
 *  trajectory_factory.hpp
 *
 *  Author(s): Tamas D. Nagy
 *	Created on: 2016-10-25
 *
 *  Static class to generate simple trajectories
 *  built of any corresponding data type.
 *
 */

#ifndef DVRK_TRAJECTORY_FACTORY_HPP_
#define DVRK_TRAJECTORY_FACTORY_HPP_

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "irob_utils/trajectory.hpp"
#include "irob_utils/utils.hpp"
#include "irob_utils/tool_pose.hpp"


namespace saf {

class TrajectoryFactory
{

private:
  TrajectoryFactory() {}
public:


  /**
   *	Uniform ramp between x1 and x2.
   */
  static Trajectory<double> uniformRamp(int N,
                                        double x1=0.0, double x2=1.0)
  {
    Trajectory<double> v;
    for (int i = 0; i < N; i++)
    {
      v.addPoint(interpolate(((double)i)/(N-1), x1, x2));
    }
    return v;
  }


  /**
   *	Accelerating ramp between x1 and x2.
   *	For decceleration: acceleratingRamp(N, x2, x1).reverse().
   */
  static Trajectory<double> acceleratingRamp(int N,
                                             double x1=0.0, double x2=1.0)
  {
    Trajectory<double> v, steps;
    double maxStep = (2.0*(x2-x1))/N;
    steps = uniformRamp(N, 0.0, maxStep);

    double a = x1;
    for (int i = 0; i < N; i++)
    {
      a += steps[i];
      v.addPoint(interpolate(a, x1, x2));
    }
    return v;
  }


  /**
   *	Smooth ramp between x1 and x2.
   */
  static Trajectory<double> smoothenedRamp(int Nfull, int Nacc, int Ndecc,
                                           double x1=0.0, double x2=1.0)
  {
    Trajectory<double> v, stepsAcc, stepsDecc;

    if (Nfull < Nacc || Nfull < Ndecc)
      throw std::runtime_error("Trajectory Nacc > Nfull");

    double accRatio = ((double)Nacc) / Nfull;
    double deccRatio = ((double)Ndecc) / Nfull;
    double maxStep = (x2-x1)/(Nfull-((Nacc+Ndecc)/2.0));
    stepsAcc = uniformRamp(Nacc, 0.0, maxStep);
    stepsDecc = uniformRamp(Ndecc, maxStep, 0.0);

    double a = x1;
    for (int i = 0; i < Nacc; i++)
    {
      a += stepsAcc[i];
      v.addPoint(interpolate(a, x1, x2));
    }

    for (int i = 0; i < Nfull-(Nacc + Ndecc); i++)
    {
      a += maxStep;
      v.addPoint(interpolate(a, x1, x2));
    }

    for (int i = 0; i < Ndecc; i++)
    {
      a += stepsDecc[i];
      v.addPoint(interpolate(a, x1, x2));
    }
    return v;
  }


  /**
   * Linear trajectory between start and end, with constant speed.
   */
  template <class P>
  static Trajectory<P> linearTrajectory(P start,P end,
                                        double speed, double dt)
  {
    Trajectory<P> tr(dt);
    double stepSize = speed * dt;
    int N = (int)round(distanceEuler(start, end) / stepSize)+1;
    Trajectory<double> ramp = uniformRamp(N);
    for (int i = 0; i < ramp.size(); i++)
    {
      tr.addPoint(interpolate(ramp[i], start, end));
    }
    return tr;
  }

  /**
   *	Smoothly accelerationg trajectory between 2 points.
   */
  template <class P>
  static Trajectory<P> linearTrajectoryWithSmoothAcceleration(
      P start,
      P end,
      double speed,
      double acc,
      double dt)
  {
    Trajectory<P> tr(dt);
    double s_full = distanceEuler(start, end);
    if (s_full < 0.00001)
      return tr;

    double t_acc = speed / acc;
    double s_acc = 0.5 * acc * t_acc * t_acc;

    double t_full = (2.0 * t_acc) + ((s_full-(2.0 * s_acc)) / speed);

    int N_full = (int)round(t_full / dt)+1;
    int N_acc = (int)round(t_acc / dt)+1;

    Trajectory<double> ramp = smoothenedRamp(N_full, N_acc, N_acc);

    for (int i = 0; i < ramp.size(); i++)
    {
      tr.addPoint(interpolate(ramp[i], start, end));
    }
    return tr;
  }


  /**
     *	Smoothly accelerationg trajectory between 2 points with waypoints.
     */
  template <class P>
  static Trajectory<P> linearTrajectoryWithSmoothAcceleration(
      P start,
      std::vector<P> waypoints,
      P end,
      double speed,
      double acc,
      double dt)
  {
    if (waypoints.empty())
      return linearTrajectoryWithSmoothAcceleration(
            start, end, speed, acc, dt);
    Trajectory<P> tr(dt);

    // To first waypoints
    double s_full = distanceEuler(start,  waypoints[0]);
    double t_acc = speed / acc;
    double s_acc = 0.5 * acc * t_acc * t_acc;
    double t_full = t_acc + ((s_full-s_acc) / speed);

    int N_full = (int)round(t_full / dt)+1;
    int N_acc = (int)round(t_acc / dt)+1;
    Trajectory<double> acc_ramp;
    try
    {
      acc_ramp = smoothenedRamp(N_full, N_acc, 0);
    }
    catch  (std::runtime_error e)
    { //acc_ramp stays empty
    }

    for (int i = 0; i < acc_ramp.size(); i++)
    {
      tr.addPoint(interpolate(acc_ramp[i], start, waypoints[0]));
    }

    // Between waypoints
    for(typename std::vector<P>::size_type i = 0;
        i < waypoints.size()-1; i++)
    {
      s_full = distanceEuler(waypoints[i],waypoints[i+1]);
      t_full = s_full / speed;

      N_full = (int)round(t_full / dt)+1;
      Trajectory<double> ramp = uniformRamp(N_full);
      for (int j = 0; j < ramp.size(); j++)
      {
        tr.addPoint(interpolate(ramp[j],
                                waypoints[i],
                                waypoints[i+1]));
      }
    }

    // To endpoint
    s_full = distanceEuler(waypoints.back(), end);
    t_acc = speed / acc;
    s_acc = 0.5 * acc * t_acc * t_acc;
    t_full = t_acc + ((s_full-s_acc) / speed);

    N_full = (int)round(t_full / dt)+1;
    N_acc = (int)round(t_acc / dt)+1;

    Trajectory<double> decc_ramp;
    try
    {
      decc_ramp = smoothenedRamp(N_full, 0, N_acc);
    }
    catch  (std::runtime_error e)
    { //decc_ramp stays empty
    }
    for (int i = 0; i < decc_ramp.size(); i++)
    {
      tr.addPoint(interpolate(decc_ramp[i],
                              waypoints.back(),
                              end));
    }

    return tr;
  }


  /**
   * Horizontal circular trajectory around center.
   */
  static Trajectory<Eigen::Vector3d> circleTrajectoryHorizontal(
      Eigen::Vector3d start,
      double toAngle, Eigen::Vector3d center,
      double T, double dt)
  {
    Trajectory<Eigen::Vector3d> tr(dt);
    int N = (int)round(T / dt)+1;
    Trajectory<double> ramp = uniformRamp(N, 0.0, toAngle);
    for (int i = 0; i < ramp.size(); i++)
    {
      Eigen::Vector3d p = start-center;
      Eigen::Vector3d p1(p.x()*cos(ramp[i])-p.y()*sin(ramp[i]),
                         p.y()*cos(ramp[i])+p.x()*sin(ramp[i]), p.z());
      Eigen::Vector3d p2 = p1+center;
      tr.addPoint(p2);
    }
    return tr;
  }


  /**
   * Horizontal circular trajectory around center.
   */
  static Trajectory<ToolPose> circleTrajectoryHorizontal(
      ToolPose start,
      double toAngle, Eigen::Vector3d center,
      double T, double dt)
  {
    Trajectory<ToolPose> tr(dt);
    int N = (int)round(T / dt)+1;
    Trajectory<double> ramp = uniformRamp(N, 0.0, toAngle);
    for (int i = 0; i < ramp.size(); i++)
    {
      Eigen::Vector3d p = start.transform.translation()-center;
      Eigen::Vector3d p1(p.x()*cos(ramp[i])-p.y()*sin(ramp[i]),
                         p.y()*cos(ramp[i])+p.x()*sin(ramp[i]), p.z());
      Eigen::Vector3d p2 = p1+center;
      ToolPose po2(p2, Eigen::Quaterniond(start.transform.rotation()), start.jaw);
      tr.addPoint(po2);
    }
    return tr;
  }


  /**
   * Vertical circular trajectory around center in Y plane.
   */
  static Trajectory<Eigen::Vector3d> circleTrajectoryVerticalY(
      Eigen::Vector3d start,
      double toAngle, Eigen::Vector3d center,
      double T, double dt)
  {
    Trajectory<Eigen::Vector3d> tr(dt);
    int N = (int)round(T / dt)+1;
    Trajectory<double> ramp = uniformRamp(N, 0.0, toAngle);
    for (int i = 0; i < ramp.size(); i++)
    {
      Eigen::Vector3d p = start-center;
      Eigen::Vector3d p1(p.x(),
                         p.y()*cos(ramp[i])+p.z()*sin(ramp[i]), p.z()*cos(ramp[i])-p.y()*sin(ramp[i]));
      Eigen::Vector3d p2 = p1+center;
      tr.addPoint(p2);
    }
    return tr;
  }


  /**
   * Vertical circular trajectory around center in X plane.
   */
  static Trajectory<Eigen::Vector3d> circleTrajectoryVerticalX(
      Eigen::Vector3d start,
      double toAngle, Eigen::Vector3d center,
      double T, double dt)
  {
    Trajectory<Eigen::Vector3d> tr(dt);
    int N = (int)round(T / dt)+1;
    Trajectory<double> ramp = uniformRamp(N, 0.0, toAngle);
    for (int i = 0; i < ramp.size(); i++)
    {
      Eigen::Vector3d p = start-center;
      Eigen::Vector3d p1(p.x()*cos(ramp[i])+p.z()*sin(ramp[i]),
                         p.y(), p.z()*cos(ramp[i])-p.x()*sin(ramp[i]));
      Eigen::Vector3d p2 = p1+center;
      tr.addPoint(p2);
    }
    return tr;
  }

};

}

#endif
