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
#include "visualization_msgs/Marker.h"


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

	ros::NodeHandle nh;
	
	Eigen::Vector3d dummy_location;
   	
   	ros::Subscriber result_sub;
	
public:
	DummyImageProcessor(ros::NodeHandle);
	~DummyImageProcessor();
	
	void subscribeTopics();
	
	void tragetCB(const visualization_msgs::MarkerConstPtr&);
	
	geometry_msgs::Point processImages( const cv_bridge::CvImagePtr, 
										const cv_bridge::CvImagePtr,
										const cv_bridge::CvImagePtr,
										const cv_bridge::CvImagePtr,
										const cv_bridge::CvImagePtr);
	

};

DummyImageProcessor::DummyImageProcessor(ros::NodeHandle nh): 
	nh(nh), dummy_location(makeNaN<Eigen::Vector3d>()) 
{
	subscribeTopics();
}

DummyImageProcessor::~DummyImageProcessor() {}


void DummyImageProcessor::subscribeTopics() 
{                 	            						
   	result_sub = nh.subscribe<visualization_msgs::Marker>(
   					"marker", 1000, 
   					&DummyImageProcessor::tragetCB,this);
}

// Callbacks
void DummyImageProcessor::tragetCB(const visualization_msgs::MarkerConstPtr& msg)
{
    dummy_location.x() = msg->pose.position.x * 1000.0;
    dummy_location.y() = msg->pose.position.y * 1000.0;
    dummy_location.z() = msg->pose.position.z * 1000.0;
}

geometry_msgs::Point DummyImageProcessor::processImages(
			const  cv_bridge::CvImagePtr image_left_ptr,
    		const cv_bridge::CvImagePtr image_right_ptr,
    		const cv_bridge::CvImagePtr color_image_left_ptr,
   			const cv_bridge::CvImagePtr color_image_right_ptr,
   			const cv_bridge::CvImagePtr disparity_ptr)
{
	ros::spinOnce();
	return wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>(dummy_location);
} 


}
#endif /* DUMMY_IMAGE_PROCESSOR_HPP_ */
