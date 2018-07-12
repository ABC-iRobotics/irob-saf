/*
 * 	accuracy_meas.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2018-07-12
 *
 * Move arm manually to any location & Wait for button press -> Save location
 * -> repeat
 *  { Move manually to another location & Wait for for button press
 *    -> Go to saved location}
 *
 */

#include <irob_subtask_logic/accuracy_meas.hpp>


namespace saf {


AccuracyMeas::AccuracyMeas(ros::NodeHandle nh, ros::NodeHandle priv_nh,
                           std::vector<std::string> arm_names):
  AutosurgAgent(nh, priv_nh, arm_names), vision(nh, "target")
{

}

AccuracyMeas::~AccuracyMeas()
{
  // TODO Auto-generated destructor stub
}



void AccuracyMeas::doAccuracyMeas()
{

    // Grasp object
    ROS_INFO_STREAM("Move arm to target position and press Enter...");

    std::cout << "Press enter to continue ...";
        std::cin.get();

    ROS_INFO_STREAM("Saving target position");
    ros::spinOnce();
    Pose target = arms[0] -> getPoseCurrent();

    ROS_INFO_STREAM("Target position saved");

    ROS_INFO_STREAM("Move arm to approach position and press Enter...");

    std::cout << "Press enter to continue ...";
        std::cin.get();

    ROS_INFO_STREAM("Saving approach position");

    ros::spinOnce();
    Pose appr = arms[0] -> getPoseCurrent();

    ROS_INFO_STREAM("Approach position saved");


    ROS_INFO_STREAM("Move arm to dist position and press Enter...");

    std::cout << "Press enter to continue ...";
        std::cin.get();

    ROS_INFO_STREAM("Saving dist position");

    ros::spinOnce();
    Pose dist = arms[0] -> getPoseCurrent();

    ROS_INFO_STREAM("Approach position saved");



    while(ros::ok())
    {


      std::cout << "Press enter to continue ...";
      std::cin.get();
      ros::spinOnce();

      ROS_INFO_STREAM("Moving arm to saved position...");

      std::vector<Pose> waypoints;

      arms[0] -> place(dist, appr, speed_cartesian, waypoints);
      while(!arms[0] -> isSurgemeDone() && ros::ok())
      {
        ros::Duration(0.1).sleep();
      }

      ROS_INFO_STREAM("Arm moved to dist position");

      ROS_INFO_STREAM("Move arm to distant position and press Enter...");

      std::cout << "Press enter to continue ...";
      std::cin.get();
      ros::spinOnce();

      ROS_INFO_STREAM("Moving arm to saved position...");


      arms[0] -> place(target, appr, speed_cartesian, waypoints);
      while(!arms[0] -> isSurgemeDone() && ros::ok())
      {
        ros::Duration(0.1).sleep();
      }

      ROS_INFO_STREAM("Arm moved to saved position");

    }
  }
}

using namespace saf;

/**
 * Maneuver server main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
  ros::init(argc, argv, "peg_transfer");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::vector<std::string> arm_names;
  priv_nh.getParam("arm_names", arm_names);


  // Start autonomous agent
  try {
    AccuracyMeas meas(nh ,  priv_nh, arm_names);

    meas.doAccuracyMeas();

    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}







