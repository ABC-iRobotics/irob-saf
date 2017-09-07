/*
 * 	stereo_sync.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-09-06
 *
 */

#ifndef STEREO_SYNC_HPP_
#define STEREO_SYNC_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace ias {

class StereoSync {
public:
  

private:

	typedef message_filters::sync_policies::ApproximateTime
			<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	
    ros::NodeHandle nh;
    
    // Subscribers must be declared before the sync,
    // else boost mutex error appears
    message_filters::Subscriber<sensor_msgs::Image> image_left_sub;
	message_filters::Subscriber<sensor_msgs::Image> image_right_sub;
    
    message_filters::Synchronizer<MySyncPolicy> sync;
   	
    


    // Publishers
    ros::Publisher image_left_pub;
    ros::Publisher image_right_pub;
    
    void advertiseTopics();

public:
	StereoSync(ros::NodeHandle);
	~StereoSync();
	


    // Callbacks
    void imageCB(
    		const sensor_msgs::ImageConstPtr&,
    		const sensor_msgs::ImageConstPtr&);
    	
	
};





}
#endif /* STEREO_SYNC_HPP_ */
