/*
 * 	surgeme_server.hpp
 * 	
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-06
 *	
 *	Separated ROS node to support preempted actions.
 *	grasp, release, cut, go_to, approach, leave...
 */

#ifndef SURGAME_SERVER_HPP_
#define SURGAME_SERVER_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <irob_utils/pose.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/utils.hpp>
#include <irob_general_robot/robot_client.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <irob_msgs/ToolPoseStamped.h>

#include <irob_msgs/SurgemeAction.h>


namespace ias {

class SurgemeServer {

public:

  struct SurgemeSetting {
    	double jaw_open_angle;
    	double jaw_closed_angle;
    	Eigen::Vector3d t;	// translation
    	
      friend std::ostream& operator<<(std::ostream&, const SurgemeSetting&);
  	};
   

protected:
	RobotClient arm;
    ros::NodeHandle nh;
    
    // Action servers
    actionlib::SimpleActionServer<irob_msgs::SurgemeAction> as;
    
    static const double DEFAULT_SPEED_CARTESIAN;	// mm/s
    static const double DEFAULT_SPEED_JAW;			// deg/s
    static const double DEFAULT_LOOP_RATE;					// Hz
	

public:
  SurgemeServer(ros::NodeHandle, std::string, double);		// dt
  ~SurgemeServer();

    // Callbacks

    void surgemeActionCB(
        const irob_msgs::SurgemeGoalConstPtr &);
    		
    Pose getPoseCurrent();
   	std::string getArmName();	
    		
protected:
		
  // Methods for surgeme execution
	void stop(); 
	  
	void nav_to_pos(Pose ,std::vector<Pose>, InterpolationMethod, double); 
	 
   	void grasp(Pose, Pose, double, double, std::vector<Pose>,
   			InterpolationMethod, 
   			double, double);
   	
   	void cut(Pose, Pose, double, std::vector<Pose>, 
   			InterpolationMethod,
   			double, double);
   			
   	void push(Pose, Pose, Eigen::Vector3d, std::vector<Pose>,
	  		InterpolationMethod,
	  		double, double);
    	
    void dissect(Pose, Pose, Eigen::Vector3d,double, std::vector<Pose>,
    		InterpolationMethod,
    		double, double); 
    		
   	void release(Pose, double, 
   			double, double);
   	
   	void place(Pose, Pose, double, std::vector<Pose>, InterpolationMethod,
   			double);
	
   	void manipulate(Eigen::Vector3d,
   			double);
   			
    bool waitForActionDone(std::string);	
	bool handleActionState(std::string, bool = false);
  bool isAbleToDoSurgeme(int);
  irob_msgs::InstrumentJawPart findInstrumentJawPartForSurgeme(int);
  SurgemeSetting calcSurgemeSetting(int, irob_msgs::InstrumentJawPart,
							Eigen::Quaternion<double>, double, double = 1.0);
   	
};

}
#endif /* SURGAME_SERVER_HPP_ */
