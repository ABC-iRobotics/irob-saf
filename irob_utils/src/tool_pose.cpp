/*
 *  tool_pose.cpp
 *
 *	Author(s): Tamas Levendovics
 *	Created on: 2017-07-08
 */


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <irob_msgs/msg/tool_pose.hpp>
#include <irob_msgs/msg/tool_pose_stamped.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

namespace saf {

class ToolPose
{
public:
  Eigen::Affine3d transform;
  double jaw;

  ToolPose() : transform(), jaw(0.0) {}

  ToolPose(double tx, double ty, double tz,
           double ow, double ox, double oy, double oz,
           double jaw)
      : transform(Eigen::Translation3d(tx, ty, tz) *
                  Eigen::Quaterniond(ow, ox, oy, oz)),
        jaw(jaw) {}

  ToolPose(const Eigen::Translation3d &t,
           const Eigen::Quaterniond &r, double jaw)
      : transform(t * r), jaw(jaw) {}

  ToolPose(const ToolPose &other)
      : transform(other.transform), jaw(other.jaw) {}

  ToolPose(const irob_msgs::msg::ToolPose &msg)
      : transform(unwrapMsg(msg.transform)), jaw(msg.jaw) {}

  ToolPose(const irob_msgs::msg::ToolPoseStamped &msg)
      : transform(unwrapMsg(msg.toolpose.transform)), jaw(msg.toolpose.jaw) {}

  ToolPose(const Eigen::Affine3d &transform, double jaw)
      : transform(transform), jaw(jaw) {}

  ToolPose(const Eigen::Vector3d &translation,
           const Eigen::Quaterniond &rotation, double jaw)
      : transform(Eigen::Translation3d(translation) * rotation), jaw(jaw) {}

  irob_msgs::msg::ToolPose toRosToolPose() const
  {
    irob_msgs::msg::ToolPose ret;
    ret.transform = wrapToMsg(transform);
    ret.jaw = jaw;
    return ret;
  }

  void swap(ToolPose &other)
  {
    ToolPose tmp(*this);
    transform = other.transform;
    jaw = other.jaw;
    other.transform = tmp.transform;
    other.jaw = tmp.jaw;
  }

  ToolPose &operator=(const ToolPose &other)
  {
    ToolPose tmp(other);
    this->swap(tmp);
    return *this;
  }

  ToolPose interpolate(double a, const ToolPose &other) const
  {
    Eigen::Quaterniond r = Eigen::Quaterniond(transform.rotation()).slerp(
        a, Eigen::Quaterniond(other.transform.rotation()));

    Eigen::Translation3d t(((1.0 - a) * transform.translation()) +
                           (a * other.transform.translation()));

    double jawd = ((1.0 - a) * jaw) + (a * other.jaw);

    return ToolPose(t, r, jawd);
  }

  bool isNaN() const
  {
    return (std::isnan(transform.translation().x()) || std::isnan(transform.translation().y())
            || std::isnan(transform.translation().z()) || std::isnan(jaw));
  }

  struct Distance
  {
    double cartesian;
    double angle;
    double jaw;

    Distance &operator*=(double d)
    {
      cartesian *= d;
      angle *= d;
      jaw *= d;
      return *this;
    }

    Distance &operator/=(double d)
    {
      cartesian /= d;
      angle /= d;
      jaw /= d;
      return *this;
    }

    Distance operator*(double d) const
    {
      Distance ret(*this);
      ret *= d;
      return ret;
    }

    Distance operator/(double d) const
    {
      Distance ret(*this);
      ret /= d;
      return ret;
    }
  };

  Distance dist(const ToolPose &other) const
  {
    Distance d;
    d.cartesian = (transform.translation() - other.transform.translation()).norm();
    double cosAlpha1_2 = Eigen::Quaterniond(transform.rotation()).dot(
        Eigen::Quaterniond(other.transform.rotation()));
    cosAlpha1_2 = std::clamp(cosAlpha1_2, -1.0, 1.0);
    d.angle = std::abs((std::acos(cosAlpha1_2) * 2.0 * 360.0) / (2.0 * M_PI));
    d.jaw = std::abs(jaw - other.jaw);
    return d;
  }

  Distance dist(const Eigen::Affine3d &other) const
  {
    Distance d;
    d.cartesian = (transform.translation() - other.translation()).norm();
    double cosAlpha1_2 = Eigen::Quaterniond(transform.rotation()).dot(
        Eigen::Quaterniond(other.rotation()));
    cosAlpha1_2 = std::clamp(cosAlpha1_2, -1.0, 1.0);
    d.angle = std::abs((std::acos(cosAlpha1_2) * 2.0 * 360.0) / (2.0 * M_PI));
    d.jaw = 0.0;
    return d;
  }

  Distance dist(const Eigen::Vector3d &otherPos) const
  {
    Distance d;
    d.cartesian = (transform.translation() - otherPos).norm();
    d.angle = 0.0;
    d.jaw = 0.0;
    return d;
  }

  Distance dist(const Eigen::Quaterniond &otherOrientation) const
  {
    Distance d;
    d.cartesian = 0.0;
    double cosAlpha1_2 = Eigen::Quaterniond(transform.rotation()).dot(otherOrientation);
    cosAlpha1_2 = std::clamp(cosAlpha1_2, -1.0, 1.0);
    d.angle = std::abs((std::acos(cosAlpha1_2) * 2.0 * 360.0) / (2.0 * M_PI));
    d.jaw = 0.0;
    return d;
  }

  Distance dist(double otherJaw) const
  {
    Distance d;
    d.cartesian = 0.0;
    d.angle = 0.0;
    d.jaw = std::abs(jaw - otherJaw);
    return d;
  }

  static Eigen::Affine3d unwrapMsg(const geometry_msgs::msg::Transform &msg)
  {
    Eigen::Affine3d transform;
    transform.translation() = Eigen::Vector3d(msg.translation.x, msg.translation.y, msg.translation.z);
    transform.linear() = Eigen::Quaterniond(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z).toRotationMatrix();
    return transform;
  }

  static geometry_msgs::msg::Transform wrapToMsg(const Eigen::Affine3d &transform)
  {
    geometry_msgs::msg::Transform msg;
    msg.translation.x = transform.translation().x();
    msg.translation.y = transform.translation().y();
    msg.translation.z = transform.translation().z();
    Eigen::Quaterniond q(transform.rotation());
    msg.rotation.w = q.w();
    msg.rotation.x = q.x();
    msg.rotation.y = q.y();
    msg.rotation.z = q.z();
    return msg;
  }
};

std::ostream &operator<<(std::ostream &os, const ToolPose &p)
{
  return os << p.transform.translation() << "\t" << p.transform.rotation() << "\t" << p.jaw;
}

std::istream &operator>>(std::istream &is, ToolPose &p)
{
  double tmp[8];
  is >> tmp[0] >> std::ws >> tmp[1] >> std::ws >> tmp[2] >> std::ws >> tmp[3]
     >> std::ws >> tmp[4] >> std::ws >> tmp[5] >> std::ws >> tmp[6] >> std::ws
     >> tmp[7] >> std::ws;
  p = ToolPose(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7]);
  return is;
}

std::ostream &operator<<(std::ostream &os, const ToolPose::Distance &d)
{
  return os << d.cartesian << "\t" << d.angle << "\t" << d.jaw;
}

} // namespace saf
