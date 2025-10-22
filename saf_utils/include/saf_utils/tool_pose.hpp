/*
 *  tool_pose.hpp
 *
 *  Author(s): Tamas Levendovics
 *  Created on: 2016-10-27
 *
 *  Class to store and make calculations on a robot arm,
 *  including the angle of the grippers.
 *
 */

#ifndef SAF_TOOL_POSE_HPP_
#define SAF_TOOL_POSE_HPP_

#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <saf_msgs/msg/tool_pose.hpp>
#include <saf_msgs/msg/tool_pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <saf_utils/utils.hpp>

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
    ToolPose(const saf_msgs::msg::ToolPose&);
    ToolPose(const saf_msgs::msg::ToolPoseStamped&);
    ToolPose(const Eigen::Vector3d&, const Eigen::Quaterniond&, double);
    ToolPose(const Eigen::Affine3d&, double );
    ToolPose(const Eigen::Translation3d&, const Eigen::Quaterniond&, double );
    ToolPose(const geometry_msgs::msg::Transform&, double);
    ToolPose(const geometry_msgs::msg::TransformStamped&, double);

    saf_msgs::msg::ToolPose toRosToolPose() const;

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
    return std::abs(d.angle);
  return std::abs(d.jaw);
}

template <>
inline ToolPose unwrapMsg(const saf_msgs::msg::ToolPose& msg){
  ToolPose ret(msg);
  return ret;
}

template <>
inline ToolPose unwrapMsg(const saf_msgs::msg::ToolPoseStamped& msg){
  ToolPose ret(msg);
  return ret;
}

template <>
inline saf_msgs::msg::ToolPose wrapToMsg(const ToolPose& data){
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