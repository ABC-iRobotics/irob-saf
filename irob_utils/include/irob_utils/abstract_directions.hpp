/*
 *  abstract_directions.hpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-08
 *  
 */

#ifndef ABSTRACT_DIRECTIONS_HPP_
#define ABSTRACT_DIRECTIONS_HPP_

#include <iostream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <cmath>
#include <irob_utils/utils.hpp>


namespace ias {

typedef enum CoordinateFrame {WORLD, CAMERA, ROBOT} CoordinateFrame;


template <CoordinateFrame CF, class T>
class AbstractDirections {
   public:
   		static const T UP;
   		static const T DOWN;
   		static const T FORWARD;
   		static const T BACKWARD;
   		static const T LEFT;
   		static const T RIGHT;  	
};

/*
 *	CAMERA
 */
 
// Eigen::Vector3d
template<> 
const Eigen::Vector3d 
		AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			UP = Eigen::Vector3d(0.0, -1.0, 0.0);
			
template<> 
const Eigen::Vector3d 
		AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			DOWN = Eigen::Vector3d(0.0, 1.0, 0.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			FORWARD = Eigen::Vector3d(0.0, 0.0, 1.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			BACKWARD = Eigen::Vector3d(0.0, 0.0, -1.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			LEFT = Eigen::Vector3d(-1.0, 0.0, 0.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			RIGHT = Eigen::Vector3d(1.0, 0.0, 0.0);
			

// geometry_msgs::Point
template<> 
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			UP 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			UP);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			DOWN 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			DOWN);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			FORWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			FORWARD);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			BACKWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			BACKWARD);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			LEFT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			LEFT);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			RIGHT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			RIGHT);

/*
 *	CAMERA
 */			
			
// Eigen::Vector3d
template<> 
const Eigen::Vector3d 
		AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			UP = Eigen::Vector3d(0.0, 0.0, 1.0);
			
template<> 
const Eigen::Vector3d 
		AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			DOWN = Eigen::Vector3d(0.0, 0.0, -1.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			FORWARD = Eigen::Vector3d(0.0, -1.0, 0.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			BACKWARD = Eigen::Vector3d(0.0, 1.0, 0.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			LEFT = Eigen::Vector3d(1.0, 0.0, 0.0);
			
template<> 
const Eigen::Vector3d
		AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			RIGHT = Eigen::Vector3d(-1.0, 0.0, 0.0);
			

// geometry_msgs::Point
template<> 
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			UP 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			UP);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			DOWN 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			DOWN);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			FORWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			FORWARD);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			BACKWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			BACKWARD);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			LEFT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			LEFT);
template<> 			
const geometry_msgs::Point 
		AbstractDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			RIGHT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(AbstractDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			RIGHT);


}
#endif
