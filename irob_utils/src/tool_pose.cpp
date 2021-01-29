/*
 *  pose.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-27
 *
 */


#include <irob_utils/tool_pose.hpp>


namespace saf {


/*
 * Constructors and basic functions for mathematical compatibility
 */
ToolPose::ToolPose(): transform(), jaw(0.0) {}

ToolPose::ToolPose(double tx, double ty, double tz,
           double ow, double ox, double oy,double oz,
           double jaw):
      transform(Eigen::Translation3d(tx, ty, tz) *
                Eigen::Quaterniond(ow, ox, oy,oz)),
      jaw(jaw) {} // TODO check

ToolPose::ToolPose(const Eigen::Translation3d& t,
                   const Eigen::Quaterniond& r,
                   double jaw):
      transform(t * r),
      jaw(jaw) {} // TODO check


ToolPose::ToolPose(const ToolPose& other):
    transform(other.transform), jaw(other.jaw) {}


ToolPose::ToolPose(const irob_msgs::ToolPose& msg):
  transform(unwrapMsg<geometry_msgs::Transform,Eigen::Affine3d>(msg.transform)),
  jaw(msg.jaw) {}

ToolPose::ToolPose(const irob_msgs::ToolPoseStamped& msg):
  transform(unwrapMsg<geometry_msgs::Transform,Eigen::Affine3d>(msg.toolpose.transform)),
  jaw(msg.toolpose.jaw) {}

ToolPose::ToolPose(const Eigen::Affine3d& transform, double jaw):
  transform(transform),
  jaw(jaw){}

ToolPose::ToolPose(const  Eigen::Vector3d& translation,
           const Eigen::Quaterniond& rotation, double jaw):
  transform(Eigen::Translation3d(translation) * Eigen::Affine3d(rotation)),
  jaw(jaw){}


irob_msgs::ToolPose ToolPose::toRosToolPose() const
{
  irob_msgs::ToolPose ret;

  ret.transform =
      wrapToMsg<geometry_msgs::Transform,Eigen::Affine3d>(transform);

  ret.jaw = jaw;
  return ret;
}



void ToolPose::swap(ToolPose& other)
{
  ToolPose tmp(*this);

  transform = other.transform;
  jaw = other.jaw;

  other.transform = tmp.transform;
  other.jaw = tmp.jaw;
}

ToolPose ToolPose::operator=(const ToolPose& other)
{
  ToolPose tmp(other);
  this->swap(tmp);
  return *this;
}


/**
 *  Pose interpolation; linear for position and jaw, SLERP for orientation.
 */
ToolPose ToolPose::interpolate(double a, const ToolPose& other) const
{

  Eigen::Quaterniond r(Eigen::Quaterniond(transform.rotation()).slerp(
                                a, Eigen::Quaterniond(
                                  other.transform.rotation())));

  Eigen::Translation3d t (((1.0-a) * transform.translation()) +
                          ((a) * other.transform.translation()));

  double jawd = ((1.0-a) * jaw) + ((a) * other.jaw);

  ToolPose res(t,r,jawd);

  return res;
}




/**
 *  Return true if any of the attributes is NaN.
 */
bool ToolPose::isNaN() const
{
  return (std::isnan(transform.translation().x()) || std::isnan(transform.translation().y())
          || std::isnan(transform.translation().z()) || std::isnan(jaw));
}


/**
 *  Distance metrics.
 */
ToolPose::Distance ToolPose::dist(const ToolPose& other) const
{
  ToolPose::Distance d;
  d.cartesian = (transform.translation() - other.transform.translation()).norm();
  double cosAlpha1_2 = Eigen::Quaterniond(transform.rotation()).dot(
                              Eigen::Quaterniond(other.transform.rotation()));
  if (cosAlpha1_2 > 1.0)
    cosAlpha1_2 = 1.0;
  else if (cosAlpha1_2 < -1.0)
    cosAlpha1_2 = -1.0;
  d.angle = std::abs((acos(cosAlpha1_2) * 2.0*360.0)/(2.0*M_PI));
  d.jaw = std::abs(jaw - other.jaw);
  return d;
}

/**
 *  Distance metrics.
 */
ToolPose::Distance ToolPose::dist(const Eigen::Affine3d& other) const
{
  ToolPose::Distance d;
  d.cartesian = (transform.translation() - other.translation()).norm();
  double cosAlpha1_2 = Eigen::Quaterniond(transform.rotation()).dot(
                              Eigen::Quaterniond(other.rotation()));
  if (cosAlpha1_2 > 1.0)
    cosAlpha1_2 = 1.0;
  else if (cosAlpha1_2 < -1.0)
    cosAlpha1_2 = -1.0;
  d.angle = std::abs((acos(cosAlpha1_2) * 2.0*360.0)/(2.0*M_PI));
  d.jaw = 0;
  return d;
}

/**
 *  Distance metrics.
 */
ToolPose::Distance ToolPose::dist(const Eigen::Vector3d& otherPos) const
{
  ToolPose::Distance d;
  d.cartesian = (transform.translation() - otherPos).norm();
  d.angle = 0.0;
  d.jaw = 0.0;
  return d;
}

/**
 *  Distance metrics.
 */
ToolPose::Distance ToolPose::dist(const Eigen::Quaterniond& otherOrientation) const
{
  ToolPose::Distance d;
  d.cartesian = 0.0;
  double cosAlpha1_2 = Eigen::Quaterniond(transform.rotation()).dot(otherOrientation);
  if (cosAlpha1_2 > 1.0)
    cosAlpha1_2 = 1.0;
  else if (cosAlpha1_2 < -1.0)
    cosAlpha1_2 = -1.0;
  d.angle = std::abs((acos(cosAlpha1_2) * 2.0*360.0)/(2.0*M_PI));
  d.jaw = 0.0;
  return d;
}

/**
 *  Distance metrics.
 */
ToolPose::Distance ToolPose::dist(double otherJaw) const
{
  ToolPose::Distance d;
  d.cartesian = 0.0;
  d.angle = 0.0;
  d.jaw = std::abs(jaw - otherJaw);
  return d;
}

/**
 *  Scaling with scalar.
 */
ToolPose::Distance ToolPose::Distance::operator*=(double d)
{
  cartesian *= d;
  angle *= d;
  jaw *= d;
  return *this;
}

/**
 *  Scaling with scalar.
 */
ToolPose::Distance ToolPose::Distance::operator/=(double d)
{
  cartesian /= d;
  angle /= d;
  jaw /= d;
  return *this;
}

/**
 *  Scaling with scalar.
 */
ToolPose::Distance ToolPose::Distance::operator*(double d) const
{
  ToolPose::Distance ret(*this);
  ret *= d;
  return ret;
}

/**
 *  Scaling with scalar.
 */
ToolPose::Distance ToolPose::Distance::operator/(double d) const
{
  ToolPose::Distance ret(*this);
  ret /= d;
  return ret;
}



/**
 *  Implementation of << operator
 */
std::ostream& operator<<(std::ostream& os, const ToolPose& p)
{
  return os << p.transform.translation() <<"\t" << p.transform.rotation() <<"\t"<< p.jaw;
}

/**
 *  Implemenation of >> operator.
 */
std::istream& operator>>(std::istream& is, ToolPose& p)
{
  double tmp[8];
  is >> tmp[0] >> std::ws >> tmp[1] >> std::ws >> tmp[2] >> std::ws >> tmp[3]
     >> std::ws >> tmp[4] >> std::ws >> tmp[5] >> std::ws >> tmp[6]>> std::ws
     >> tmp[7] >> std::ws;

  p = ToolPose(tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6],tmp[7]);
  return is;
}

/**
 *  Implementation of << operator for Distance.
 */
std::ostream& operator<<(std::ostream& os, const ToolPose::Distance& d)
{
  return os << d.cartesian <<"\t" << d.angle <<"\t"
            << d.jaw;
}

/**
 *  Rotate using rotation matrix.
 */
ToolPose operator*(const Eigen::Matrix3d& R, const ToolPose& p)
{
  ToolPose ret(R * p.transform, p.jaw);
  return ret;
}

ToolPose operator*(const Eigen::Affine3d& T, const ToolPose& p)
{
  ToolPose ret(T * p.transform, p.jaw);
  return ret;
}






}



















