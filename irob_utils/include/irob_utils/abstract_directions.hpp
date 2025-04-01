/*
 *  abstract_directions.hpp
 *
 *  Author(s): Tamas Levendovics
 *  Updated for ROS 2
 */

#ifndef ABSTRACT_DIRECTIONS_HPP_
#define ABSTRACT_DIRECTIONS_HPP_

#include <iostream>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <irob_utils/utils.hpp>

namespace saf {

enum class CoordinateFrame {WORLD, CAMERA, ROBOT};

template <CoordinateFrame CF, class T>
class BaseDirections {
   public:
        static const T UP;
        static const T DOWN;
        static const T FORWARD;
        static const T BACKWARD;
        static const T LEFT;
        static const T RIGHT;
};

/* CAMERA Directions */
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::UP = Eigen::Vector3d(0.0, -1.0, 0.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::DOWN = Eigen::Vector3d(0.0, 1.0, 0.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::FORWARD = Eigen::Vector3d(0.0, 0.0, 1.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::BACKWARD = Eigen::Vector3d(0.0, 0.0, -1.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::LEFT = Eigen::Vector3d(-1.0, 0.0, 0.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::RIGHT = Eigen::Vector3d(1.0, 0.0, 0.0);

/* ROBOT Directions */
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::UP = Eigen::Vector3d(0.0, 0.0, 1.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::DOWN = Eigen::Vector3d(0.0, 0.0, -1.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::FORWARD = Eigen::Vector3d(0.0, -1.0, 0.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::BACKWARD = Eigen::Vector3d(0.0, 1.0, 0.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::LEFT = Eigen::Vector3d(1.0, 0.0, 0.0);
template<>
const Eigen::Vector3d BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::RIGHT = Eigen::Vector3d(-1.0, 0.0, 0.0);

/* ROBOT Orientations */
template <CoordinateFrame CF, class T>
class BaseOrientations {
   public:
        static const T UP_FORWARD;
        static const T UP_SIDEWAYS;
        static const T DOWN_FORWARD;
        static const T DOWN_SIDEWAYS;
        static const T FORWARD_HORIZONTAL;
        static const T FORWARD_VERTICAL;
        static const T BACKWARD_HORIZONTAL;
        static const T BACKWARD_VERTICAL;
        static const T LEFT_HORIZONTAL;
        static const T LEFT_VERTICAL;
        static const T RIGHT_HORIZONTAL;
        static const T RIGHT_VERTICAL;
};

template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::UP_FORWARD = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::UP_SIDEWAYS = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);
template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::DOWN_FORWARD = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);
template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::DOWN_SIDEWAYS = Eigen::Quaterniond(0.0, -0.7071, 0.7071, 0.0);

template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::FORWARD_HORIZONTAL = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);
template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::FORWARD_VERTICAL = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::RIGHT_HORIZONTAL = Eigen::Quaterniond(0.0, 0.7071, 0.0, -0.7071);
template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::RIGHT_VERTICAL = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);

template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::LEFT_HORIZONTAL = Eigen::Quaterniond(0.0, -0.7071, 0.0, 0.7071);
template<>
const Eigen::Quaterniond BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaterniond>::LEFT_VERTICAL = Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);

}  // namespace saf

#endif  // ABSTRACT_DIRECTIONS_HPP_