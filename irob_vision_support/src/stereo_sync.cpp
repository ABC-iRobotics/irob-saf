/*
 * 	stereo_sync.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-09-06
 *  
 */





#include <irob_vision_support/stereo_sync.hpp>


namespace ias {

StereoSync::StereoSync(ros::NodeHandle nh): nh(nh), 
			image_left_sub(nh, "left/image", 1),
			image_right_sub(nh, "right/image", 1),
			sync(MySyncPolicy(10), image_left_sub, image_right_sub)
{
	advertiseTopics();
  	// ApproximateTime takes a queue size as 
  	// its constructor argument, hence 
  	// MySyncPolicy(10)
  					
  	sync.registerCallback(boost::bind(&StereoSync::imageCB, this, _1, _2));
}

StereoSync::~StereoSync() {}


void StereoSync::advertiseTopics() 
{
	image_left_pub = 
		nh.advertise<sensor_msgs::Image>("left/synced/image", 1000); 

	image_right_pub = 
		nh.advertise<sensor_msgs::Image>("right/synced/image", 1000);    
}

void StereoSync::imageCB(
    		const sensor_msgs::ImageConstPtr& image_left,
    		const sensor_msgs::ImageConstPtr& image_right)
{		
	image_left_pub.publish(*image_left);
	image_right_pub.publish(*image_right);
}







}




using namespace ias;

/**
 * Image sync main 
 */
int main(int argc, char **argv)
{
	
	// Initialize ros node
    ros::init(argc, argv, "stereo_sync");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    
    
    // Start Vision server
  	try {
    	StereoSync sync(nh);	
    	ros::spin();
    	
    	ROS_INFO_STREAM("Program finished succesfully, shutting down ...");
    	
    } catch (const std::exception& e) {
  		ROS_ERROR_STREAM(e.what());
  		ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  	}
    
    
    // Exit
    ros::shutdown();
	return 0;
}




























