/*
 *  optoforce_listener.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2016-10-10
 *  
 */

#include <irob_sensory_support/optoforce_client.hpp>
#include <numeric>
#include <chrono>
#include <cmath>

namespace saf {

// TODO check topic name
const std::string OptoforceClient::TOPIC_NAME
                    = "/optoforce_sensor";

OptoforceClient::OptoforceClient(
			ros::NodeHandle nh): nh(nh)
{

	// Subscribe and advertise topics
	subscribe(TOPIC_NAME);
	offsets << -434.0, -297.0, 28622.0;
  forces << std::nan(""),  std::nan(""),  std::nan("");
      
}

OptoforceClient::~OptoforceClient()
{
	// TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void OptoforceClient::forcesCB(optoforcesensor::FT msg)
{
    forces << msg.FT.Fx-offsets.x(), msg.FT.Fy-offsets.y(), msg.FT.Fz-offsets.z();
}


bool OptoforceClient::subscribe(std::string topic)
{
    forces_sub = nh.subscribe<optoforcesensor::FT>(
                        topic, 1000,
                        &OptoforceClient::forcesCB,this);
    
    ROS_INFO_STREAM("Subscribed to topic " << topic);
    return true;
}


Eigen::Vector3d OptoforceClient::getForcesCurrent()
{
	ros::spinOnce();
 	return forces;
}

void OptoforceClient::calibrateOffsets()
{

	double nSamples = 5;
	Eigen::Vector3d tmp(0.0, 0.0, 0.0);
	ros::Rate loop_rate(50.0);
	for (int i = 0; i < nSamples; i++)
	{
    Eigen::Vector3d f = getForcesCurrent();
    tmp += f;
		loop_rate.sleep();
	}
	tmp /= nSamples;
	offsets += tmp;
}


}













