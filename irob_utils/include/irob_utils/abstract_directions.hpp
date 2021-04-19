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


namespace saf {

typedef enum CoordinateFrame {WORLD, CAMERA, ROBOT} CoordinateFrame;


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

/*
 *	CAMERA
 */
 
// Eigen::Vector3d
template<> 
const Eigen::Vector3d 
		BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			UP = Eigen::Vector3d(0.0, -1.0, 0.0);
			
template<> 
const Eigen::Vector3d 
		BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			DOWN = Eigen::Vector3d(0.0, 1.0, 0.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			FORWARD = Eigen::Vector3d(0.0, 0.0, 1.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			BACKWARD = Eigen::Vector3d(0.0, 0.0, -1.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			LEFT = Eigen::Vector3d(-1.0, 0.0, 0.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			RIGHT = Eigen::Vector3d(1.0, 0.0, 0.0);
			

// geometry_msgs::Point
template<> 
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			UP 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			UP);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			DOWN 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			DOWN);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			FORWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			FORWARD);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			BACKWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			BACKWARD);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			LEFT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			LEFT);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::CAMERA, geometry_msgs::Point>::
			RIGHT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::CAMERA, Eigen::Vector3d>::
			RIGHT);

/*
 *	ROBOT
 */			
			
// Eigen::Vector3d
template<> 
const Eigen::Vector3d 
		BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			UP = Eigen::Vector3d(0.0, 0.0, 1.0);
			
template<> 
const Eigen::Vector3d 
		BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			DOWN = Eigen::Vector3d(0.0, 0.0, -1.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			FORWARD = Eigen::Vector3d(0.0, -1.0, 0.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			BACKWARD = Eigen::Vector3d(0.0, 1.0, 0.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			LEFT = Eigen::Vector3d(1.0, 0.0, 0.0);
			
template<> 
const Eigen::Vector3d
		BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			RIGHT = Eigen::Vector3d(-1.0, 0.0, 0.0);
			

// geometry_msgs::Point
template<> 
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			UP 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			UP);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			DOWN 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			DOWN);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			FORWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			FORWARD);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			BACKWARD 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			BACKWARD);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			LEFT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			LEFT);
template<> 			
const geometry_msgs::Point 
		BaseDirections<CoordinateFrame::ROBOT, geometry_msgs::Point>::
			RIGHT 
			= wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>
			(BaseDirections<CoordinateFrame::ROBOT, Eigen::Vector3d>::
			RIGHT);


/*
 * --------------------------------------------------------------------------
 */

template <CoordinateFrame CF, class T>
class BaseOrientations {
   public:
   		// The first word is corresponding to the main direction,
   		// the second to the rotation of the tool axis -- the open/close
   		// direction of the jaws
   		
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

/*
 *	ROBOT
 */			
			
// Eigen::Quaternion<double>
template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			UP_FORWARD = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);
			
template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			UP_SIDEWAYS = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);


template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			DOWN_FORWARD = Eigen::Quaternion<double>(0.0, 1.0, 0.0, 0.0);

template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			DOWN_SIDEWAYS = 
      Eigen::Quaternion<double>(0.0, -0.7071, 0.7071, 0.0);


template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			FORWARD_HORIZONTAL = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);	
				
template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			FORWARD_VERTICAL = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);


template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			BACKWARD_HORIZONTAL = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);	
				
template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			BACKWARD_VERTICAL = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);


template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			RIGHT_HORIZONTAL = 
			Eigen::Quaternion<double>(0.0, 0.7071, 0.0, -0.7071);	
			
template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			RIGHT_VERTICAL = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);
			

template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			LEFT_HORIZONTAL = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);	
			
template<> 
const Eigen::Quaternion<double> 
		BaseOrientations<CoordinateFrame::ROBOT, Eigen::Quaternion<double>>::
			LEFT_VERTICAL = Eigen::Quaternion<double>(0.0, 0.0, 0.0, 0.0);	

	
			


}
#endif
