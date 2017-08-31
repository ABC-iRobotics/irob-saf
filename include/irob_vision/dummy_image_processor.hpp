/*
 * 	dummy_image_processor.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-08-31
 *
 */

#ifndef DUMMY_IMAGE_PROCESSOR_HPP_
#define DUMMY_IMAGE_PROCESSOR_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/Point.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "irob_utils/pose.hpp"
#include "irob_utils/utils.hpp"


namespace ias {

class DummyImageProcessor {

private:

	Eigen::Vector3d dummy_location;
	
public:
	DummyImageProcessor();
	~DummyImageProcessor();
	
	geometry_msgs::Point processImages( const cv_bridge::CvImagePtr, 
										const cv_bridge::CvImagePtr,
										const cv_bridge::CvImagePtr,
										const cv_bridge::CvImagePtr,
										const cv_bridge::CvImagePtr);
	

};

DummyImageProcessor::DummyImageProcessor(): dummy_location(0.0, 0.0, -120.0) {}

DummyImageProcessor::~DummyImageProcessor() {}



geometry_msgs::Point DummyImageProcessor::processImages(
			const  cv_bridge::CvImagePtr image_left_ptr,
    		const cv_bridge::CvImagePtr image_right_ptr,
    		const cv_bridge::CvImagePtr color_image_left_ptr,
   			const cv_bridge::CvImagePtr color_image_right_ptr,
   			const cv_bridge::CvImagePtr disparity_ptr)
{
	return wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>(dummy_location);
} 


}
#endif /* DUMMY_IMAGE_PROCESSOR_HPP_ */
