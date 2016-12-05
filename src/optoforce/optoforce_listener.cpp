/*
 *  optoforce_listener.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *  
 */

#include "optoforce/optoforce_listener.hpp"
#include <numeric>
#include <chrono>

namespace optoforce {

// TODO check topic name
const std::string OptoforceListener::TOPIC_NAME
                    = "/optoforce_sensor";

OptoforceListener::OptoforceListener(
			ros::NodeHandle nh): nh(nh)
{

	// Subscribe and advertise topics
	subscribe(TOPIC_NAME);
	offsets << -434.0, -297.0, 28622.0;
      
}

OptoforceListener::~OptoforceListener()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void OptoforceListener::forcesCB(optoforcesensor::FT msg) 
{
    forces << msg.FT.Fx-offsets.x(), msg.FT.Fy-offsets.y(), msg.FT.Fz-offsets.z();
}


bool OptoforceListener::subscribe(std::string topic) 
{
    forces_sub = nh.subscribe<optoforcesensor::FT>(
                        topic, 1000,
                        &OptoforceListener::forcesCB,this);
    
    ROS_INFO_STREAM("Subscribed to topic " << topic);
    return true;
}


Eigen::Vector3d OptoforceListener::getForcesCurrent()
{
	ros::spinOnce();
 	return forces;
}

void OptoforceListener::calibrateOffsets()
{
	double nSamples = 5;
	Eigen::Vector3d tmp(0.0, 0.0, 0.0);
	ros::Rate loop_rate(50.0);
	for (int i = 0; i < nSamples; i++)
	{
		tmp += getForcesCurrent();
		loop_rate.sleep();
	}
	tmp /= nSamples;
	offsets += tmp;
}


}













