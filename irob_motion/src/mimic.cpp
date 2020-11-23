/*
 *  mimic.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-07-19
 *
 */

#include <irob_motion/mimic.hpp>

namespace saf {


const double Mimic::DEFAULT_LOOP_RATE = 10.0;				// Hz


Mimic::Mimic(ros::NodeHandle nh, std::string primer_name, std::string seconder_name,
                             double dt):
  nh(nh), primer_arm(nh, primer_name, dt),  seconder_arm(nh, seconder_name, dt)

{

}

Mimic::~Mimic()
{
  // TODO Auto-generated destructor stub
}


void Mimic::mime()
{
  ros::Rate loop_rate(Mimic::DEFAULT_LOOP_RATE);
  while(ros::ok())
  {
      ros::spinOnce();
      sensor_msgs::JointState joint_state = primer_arm.getJointStateCurrent();
      //ROS_INFO_STREAM(joint_state);
      seconder_arm.moveJoints(joint_state);
      loop_rate.sleep();
  }

}


}

using namespace saf;


/**
 * Mimic main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
  ros::init(argc, argv, "mimic");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string primer_name;
  priv_nh.getParam("primer_name", primer_name);

  std::string seconder_name;
  priv_nh.getParam("seconder_name", seconder_name);

  double rate_command;
  priv_nh.getParam("rate", rate_command);


  // Start surgeme server
  try {
    Mimic mimic(nh, primer_name, seconder_name, 1.0/rate_command);

    mimic.mime();

    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}


