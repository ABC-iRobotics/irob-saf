/*
 * 	camera.cpp
 *
 *	Author(s): Cecilia Molnar
 *	Created on: 2019-08-22
 *
 */

#include <irob_subtask_logic/camera.hpp>


namespace saf {


Camera::Camera(ros::NodeHandle nh, ros::NodeHandle priv_nh,
             std::vector<std::string> arm_names, double speed_carthesian):
  AutosurgAgent(nh, priv_nh, arm_names), vision(nh, "target"), speed_carthesian(speed_carthesian)
{	
  //
}

Camera::~Camera()
{
  // TODO Auto-generated destructor stub
}


void Camera::moveCam()
{

  // NaN pose received, until the vision node starts
  Eigen::Vector3d p = makeNaN<Eigen::Vector3d>();





  // Send grasp surgeme action to the surgeme server.
  while (ros::ok()){

     double thresh_zoom=0.5;
     p = vision.getResult();
     //if(!isnan(p)){ROS_INFO_STREAM("z abs...: "<< fabs(p(2)));}
     double xy= sqrt(p(0)*p(0) + p(1)*p(1));
     double thresh_zoom_upper = 0.5;
     double thresh_zoom_lower = 0.1;
     if(!isnan(p) && xy>=thresh_zoom && (p(2)>=thresh_zoom_upper || p(2)<thresh_zoom_lower))
        {
         ROS_INFO_STREAM("Marker displacement received: " << p);

         ROS_INFO_STREAM("Start moving maneuver...");
         arms[0] -> move_cam(p, speed_carthesian);


           // Wait for action to be finished
         while(!arms[0] -> isSurgemeDone() && ros::ok())
             {

                // Receive action result
                p = vision.getResult();
                ros::Duration(0.1).sleep();
             }
         }

     if(!isnan(p) && xy>=thresh_zoom && (p(2)<thresh_zoom_upper && p(2)>=thresh_zoom_lower))
        {
         p(2)=0;
         ROS_INFO_STREAM("Marker displacement received: " << p);

         ROS_INFO_STREAM("Start moving maneuver...");
         arms[0] -> move_cam(p, speed_carthesian);


           // Wait for action to be finished
         while(!arms[0] -> isSurgemeDone() && ros::ok())
             {

                // Receive action result
                p = vision.getResult();
                ros::Duration(0.1).sleep();
             }
         }

     if(!isnan(p) && xy<thresh_zoom && (p(2)>=thresh_zoom_upper || p(2)<thresh_zoom_lower))
        {
         p(0)=0;
         p(1)=0;
         ROS_INFO_STREAM("Marker displacement received: " << p);

         ROS_INFO_STREAM("Start moving maneuver...");
         arms[0] -> move_cam(p, speed_carthesian);


           // Wait for action to be finished
         while(!arms[0] -> isSurgemeDone() && ros::ok())
             {

                // Receive action result
                p = vision.getResult();
                ros::Duration(0.1).sleep();
             }
         }
     }

  ROS_INFO_STREAM("Camera moving succeeded");
}

}

using namespace saf;

/**
 * Maneuver server main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
  ros::init(argc, argv, "dummy_grasp");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::vector<std::string> arm_names;
  priv_nh.getParam("arm_names", arm_names);
  double speed_carthesian;
  priv_nh.getParam("speed_cartesian", speed_carthesian);



  // Start autonomous agent
  try {
    Camera pnp(nh, priv_nh, arm_names, speed_carthesian);

    pnp.moveCam();

    ROS_INFO_STREAM("Program finished succesfully, shutting down ...");

  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_ERROR_STREAM("Program stopped by an error, shutting down ...");
  }


  // Exit
  ros::shutdown();
  return 0;
}



























