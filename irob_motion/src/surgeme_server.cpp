/*
 *  surgeme_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-06
 *
 */

#include <irob_motion/surgeme_server.hpp>

namespace ias {


const double SurgemeServer::DEFAULT_LOOP_RATE = 10.0;				// Hz
const double  SurgemeServer::DEFAULT_SPEED_CARTESIAN = 30.0;	// mm/s
const double SurgemeServer::DEFAULT_SPEED_JAW = 1.0;			// deg/s


SurgemeServer::SurgemeServer(ros::NodeHandle nh, std::string arm_name, 
                             double dt):
  nh(nh), arm(nh, arm_name, dt),
  as(nh, "surgeme/"+arm_name, boost::bind(
       &SurgemeServer::surgemeActionCB, this, _1), false)
{

  // Subscribe and advertise topics

  as.start();
  arm.initArm(true);
}

SurgemeServer::~SurgemeServer()
{
  // TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */
void SurgemeServer::surgemeActionCB(
    const irob_msgs::SurgemeGoalConstPtr &goal)
{	
  // Extract data from msg
  InterpolationMethod	interp_method;
  if (goal -> interpolation == goal -> INTERPOLATION_BEZIER)
    interp_method = InterpolationMethod::BEZIER;
  else
    interp_method = InterpolationMethod::LINEAR;

  Pose target(goal -> target, 0.0);
  Pose approach_pose(goal -> approach_pose, 0.0);
  std::vector<Pose> waypoints;
  for (geometry_msgs::Pose p : goal->waypoints)
    waypoints.push_back(Pose(p, 0.0));

  Eigen::Vector3d displacement(goal->displacement.x,
                               goal->displacement.y,
                               goal->displacement.z);
  // Check tool
  if (!isAbleToDoSurgeme(goal -> action))
  {
    irob_msgs::SurgemeResult result;

    ROS_ERROR_STREAM(
          "Current instrument cannot be used for the action, aborting surgeme");
    result.pose = arm.getPoseCurrent().toRosToolPose();
    result.info = "instrument issue";

    as.setAborted(result);
    return;
  }


  try {
    // Do the action
    switch(goal -> action)
    {
    // STOP
    case irob_msgs::SurgemeGoal::STOP:
    {
      stop();
      break;
    }

      // NAV_TO_POS
    case irob_msgs::SurgemeGoal::NAV_TO_POS:
    {
      nav_to_pos(target, waypoints,
                 interp_method, goal -> speed_cartesian);
      break;
    }

      // GRASP
    case irob_msgs::SurgemeGoal::GRASP:
    {
      grasp(target, approach_pose,
            goal -> target_diameter,
            goal -> compression_rate,
            waypoints, interp_method,
            goal -> speed_cartesian, goal -> speed_jaw);
      break;
    }

      // CUT
    case irob_msgs::SurgemeGoal::CUT:
    {
      cut(target, approach_pose,
          goal -> target_diameter,
          waypoints, interp_method,
          goal -> speed_cartesian, goal -> speed_jaw);
      break;
    }

      // PUSH
    case irob_msgs::SurgemeGoal::PUSH:
    {
      push(target, approach_pose,
           displacement,
           waypoints, interp_method,
           goal -> speed_cartesian, goal -> speed_jaw);
      break;
    }

      // DISSECT
    case irob_msgs::SurgemeGoal::DISSECT:
    {
      dissect(target, approach_pose,
              displacement, goal -> target_diameter,
              waypoints, interp_method,
              goal -> speed_cartesian, goal -> speed_jaw);
      break;
    }

      // PLACE
    case irob_msgs::SurgemeGoal::PLACE:
    {
      place( target, approach_pose, goal -> target_diameter,
             waypoints, interp_method, goal -> speed_cartesian);
      break;
    }

      // MANIPULATE
    case irob_msgs::SurgemeGoal::MANIPULATE:
    {
      manipulate(displacement, goal -> speed_cartesian);
      break;
    }

      // RELEASE
    case irob_msgs::SurgemeGoal::RELEASE:
    {
      release(approach_pose, goal -> target_diameter,
              goal -> speed_cartesian, goal -> speed_jaw);
      break;
    }
    default:
    {
      ROS_ERROR_STREAM(arm.getName()  <<
                       ": invalid surgeme action code received");
      irob_msgs::SurgemeResult result;
      result.pose = arm.getPoseCurrent().toRosToolPose();
      result.info = arm.getName()  +
          ": invalid surgeme action code";
      as.setAborted(result);
      break;
    }
    }
  } catch (std::runtime_error e) {
    std::string str_abort = "abort";
    std::string str_err = e.what();
    if (str_err.find(str_abort) != std::string::npos)
      ROS_ERROR_STREAM(e.what());
    else
      throw e;
  }
}



// Helper methods

/**
 *	Wait for actionDone()
 */
bool SurgemeServer::waitForActionDone(std::string stage)
{
  bool done = false;
  ros::Rate loop_rate(DEFAULT_LOOP_RATE);

  irob_msgs::SurgemeFeedback feedback;

  while(!done)
  {
    // Check that preempt has not been requested by the client
    if (as.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO_STREAM(arm.getName()  << ": " << stage << ": Preempted");
      // Set the action state to preempted
      as.setPreempted();
      break;
    }

    // Send some feedback
    feedback.pose = arm.getPoseCurrent().toRosToolPose();
    feedback.info = stage;
    as.publishFeedback(feedback);

    done = arm.isActionDone();
    loop_rate.sleep();
  }

  return done;
}

/**
 *	Evaluate action state
 */
bool SurgemeServer::handleActionState(std::string stage, 
                                      bool lastStage /* = false */ )
{
  irob_msgs::SurgemeResult result;
  irob_msgs::SurgemeFeedback feedback;

  switch (arm.getState().state_)
  {
  case actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED:
    ROS_INFO_STREAM(arm.getName()  << ": " << stage << " succeeded");
    feedback.pose = arm.getPoseCurrent().toRosToolPose();
    feedback.info = stage + " succeeded";
    as.publishFeedback(feedback);

    // Do not send succeeded if not the last stage,
    // else the surgeme continues
    if (lastStage)
    {
      result.pose = arm.getPoseCurrent().toRosToolPose();
      result.info = stage + " succeeded";
      as.setSucceeded(result);
    }

    break;
  case actionlib::SimpleClientGoalState::StateEnum::ABORTED:
    // Send ABORTED to the upper level node
    // and throw an error to finish surgeme execution
    ROS_ERROR_STREAM(arm.getName()  << ": " << stage << " aborted");
    result.pose = arm.getPoseCurrent().toRosToolPose();
    result.info = stage + " aborted";

    as.setAborted(result);

    throw std::runtime_error(
          "Action aborted by node on a lower level.");
    break;
  default:
    // PREEMPTED or similar, do nothing
    break;
  }
}

bool SurgemeServer::isAbleToDoSurgeme(int surgeme_type)
{
  if (	surgeme_type == irob_msgs::SurgemeGoal::STOP
        || 	surgeme_type == irob_msgs::SurgemeGoal::NAV_TO_POS
        ||	surgeme_type == irob_msgs::SurgemeGoal::PUSH
        || 	surgeme_type == irob_msgs::SurgemeGoal::RELEASE)
    return true;

  if (arm.getInstrumentInfo().basic_type == irob_msgs::InstrumentInfo::GRIPPER
      && (	surgeme_type == irob_msgs::SurgemeGoal::GRASP
            ||	surgeme_type == irob_msgs::SurgemeGoal::DISSECT
            ||	surgeme_type == irob_msgs::SurgemeGoal::PLACE
            ||	surgeme_type == irob_msgs::SurgemeGoal::MANIPULATE))
    return true;

  if (arm.getInstrumentInfo().basic_type == irob_msgs::InstrumentInfo::SCISSORS
      && (	surgeme_type == irob_msgs::SurgemeGoal::CUT))
    return true;

  if(findInstrumentJawPartForSurgeme(surgeme_type).type
     != irob_msgs::InstrumentJawPart::JOINT)
    return true;

  return false;
}

irob_msgs::InstrumentJawPart SurgemeServer::findInstrumentJawPartForSurgeme(
    int surgeme_type)
{
  // Return a JOINT type instruemtJawPart if not found
  irob_msgs::InstrumentJawPart ret;
  ret.type = irob_msgs::InstrumentJawPart::JOINT;
  ret.start = 0.0;
  ret.end = 0.0;

  for (irob_msgs::InstrumentJawPart p : arm.getInstrumentInfo().jaw_parts)
  {
    if (	surgeme_type == irob_msgs::SurgemeGoal::STOP
          || 	surgeme_type == irob_msgs::SurgemeGoal::NAV_TO_POS
          ||	surgeme_type == irob_msgs::SurgemeGoal::PUSH
          || 	surgeme_type == irob_msgs::SurgemeGoal::RELEASE)
      ret = p;

    else if (p.type == irob_msgs::InstrumentJawPart::GRIPPER
             && (	surgeme_type == irob_msgs::SurgemeGoal::GRASP
                  ||	surgeme_type == irob_msgs::SurgemeGoal::DISSECT
                  ||	surgeme_type == irob_msgs::SurgemeGoal::PLACE
                  ||	surgeme_type == irob_msgs::SurgemeGoal::MANIPULATE))
      ret = p;

    else if (p.type == irob_msgs::InstrumentJawPart::SCISSORS
             && (	surgeme_type == irob_msgs::SurgemeGoal::CUT))
      ret = p;
  }

  return ret;
}

SurgemeServer::SurgemeSetting SurgemeServer::calcSurgemeSetting(
    int surgeme_type,
    irob_msgs::InstrumentJawPart jaw_part,
    Eigen::Quaternion<double> target_ori,
    double target_diameter,
    double compression_rate /* = 1.0 */)
{
  SurgemeSetting g;
  g.jaw_open_angle = 0.0;
  g.jaw_closed_angle = 0.0;
  g.t = Eigen::Vector3d(0.0, 0.0, 0.0);

  // Does not matters
  if (	surgeme_type == irob_msgs::SurgemeGoal::STOP
        || 	surgeme_type == irob_msgs::SurgemeGoal::NAV_TO_POS
        ||	surgeme_type == irob_msgs::SurgemeGoal::MANIPULATE )
    return g;

  // Center of jaw part
  if (	surgeme_type == irob_msgs::SurgemeGoal::GRASP
        || 	surgeme_type == irob_msgs::SurgemeGoal::PLACE
        ||  surgeme_type == irob_msgs::SurgemeGoal::RELEASE)
  {

    double dist = jaw_part.end - (target_diameter / 2.0);

    g.t = -1.0 * dist * quatToVec<Eigen::Quaternion<double>,
        Eigen::Vector3d>(target_ori);
    g.t *= 1.2;

    g.jaw_closed_angle = (2.0 * atan(((target_diameter / 2.0)
                                      * compression_rate) / dist))
        * (180.0 / M_PI);

    g.jaw_open_angle = (2.0 * atan(((target_diameter / 2.0)
                                    * 1.8) / dist))
        * (180.0 / M_PI);

    return g;
  }

  if (surgeme_type == irob_msgs::SurgemeGoal::CUT)
  {
    double dist = jaw_part.end - (target_diameter / 2.0);

    g.t = -1.0 * dist * quatToVec<Eigen::Quaternion<double>,
        Eigen::Vector3d>(target_ori);

    g.jaw_open_angle = (2.0 * atan((target_diameter / 2.0) / dist))
        * (180.0 / M_PI);

    g.jaw_closed_angle = 0.0;
    return g;
  }

  // End of jaw
  // TODO check
  if (	surgeme_type == irob_msgs::SurgemeGoal::DISSECT
        ||	surgeme_type == irob_msgs::SurgemeGoal::PUSH)
  {
    g.jaw_open_angle = (2.0 * (target_diameter
                               / (2.0 * arm.getInstrumentInfo().jaw_length)))
        * (180.0 / M_PI);

    double dist = arm.getInstrumentInfo().jaw_length
        * cos(g.jaw_open_angle * (M_PI / 360.0));

    g.t = -1.0 * dist * quatToVec<Eigen::Quaternion<double>,
        Eigen::Vector3d>(target_ori);
    g.jaw_closed_angle = 0.0;
    return g;
  }


}



/**
 * -----------------------------------------------------------------------------
 * Actions
 * -----------------------------------------------------------------------------
 */


/**
 * Stop
 */
void SurgemeServer::stop()
{
  // Helper variables
  bool done = false;
  std::string stage = "stop";

  // Start action
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.stop();

  done = waitForActionDone(stage);

  if (done)
    handleActionState(stage, true);
  else
    return;

}

/**
 * Nav_to_pos
 */
void SurgemeServer::nav_to_pos(Pose target, std::vector<Pose> waypoints,
                               InterpolationMethod interp_method, double speed_cartesian)
{

  // Helper variables
  bool done = false;
  std::string stage = "navigate";

  // Start action
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(target, speed_cartesian, waypoints, interp_method);

  done = waitForActionDone(stage);

  if (done)
    handleActionState(stage, true);
  else
    return;
}

/**
 * Grasp
 */
void SurgemeServer::grasp(Pose target, Pose approach_pose,
                          double target_diameter,
                          double compression_rate,
                          std::vector<Pose> waypoints, InterpolationMethod interp_method,
                          double speed_cartesian, double speed_jaw)
{
  // Helper variables
  bool done = false;
  std::string stage = "";

  irob_msgs::InstrumentJawPart jaw_part =
      findInstrumentJawPartForSurgeme(irob_msgs::SurgemeGoal::GRASP);

  SurgemeSetting gs = calcSurgemeSetting(irob_msgs::SurgemeGoal::GRASP,
                                         jaw_part,
                                         target.orientation,
                                         target_diameter,
                                         compression_rate);


  // Start action

  // Navigate
  stage = "navigate";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage << " at "<< target);
  arm.moveTool(approach_pose + gs.t,
               speed_cartesian, waypoints, interp_method);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Open tool
  stage = "open_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_open_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Approach
  stage = "approach";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(target + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Close tool
  stage = "close_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_closed_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, true);
  else
    return;
}

/**
 * Cut
 */
void SurgemeServer::cut(Pose target, Pose approach_pose,
                        double target_diameter,
                        std::vector<Pose> waypoints, InterpolationMethod interp_method,
                        double speed_cartesian, double speed_jaw)
{
  // Helper variables
  bool done = false;
  std::string stage = "";

  irob_msgs::InstrumentJawPart jaw_part =
      findInstrumentJawPartForSurgeme(irob_msgs::SurgemeGoal::CUT);

  SurgemeSetting gs = calcSurgemeSetting(irob_msgs::SurgemeGoal::CUT,
                                         jaw_part, target.orientation,
                                         target_diameter);

  // Start action

  // Navigate
  stage = "navigate";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(approach_pose + gs.t, speed_cartesian, waypoints, interp_method);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Open tool
  stage = "open_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_open_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Approach
  stage = "approach";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(target + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Close tool
  stage = "close_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_closed_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, true);
  else
    return;
}

/**
 * Release
 */
void SurgemeServer::release(Pose approach_pose,
                            double target_diameter,
                            double speed_cartesian, double speed_jaw)
{
  // Helper variables
  bool done = false;
  std::string stage = "";

  irob_msgs::InstrumentJawPart jaw_part =
      findInstrumentJawPartForSurgeme(irob_msgs::SurgemeGoal::RELEASE);

  SurgemeSetting gs = calcSurgemeSetting(irob_msgs::SurgemeGoal::RELEASE,
                                         jaw_part, approach_pose.orientation,
                                         target_diameter);

  // Start action

  // Open tool
  stage = "open_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_open_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Approach
  stage = "leave";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(approach_pose + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, true);
  else
    return;

}

/**
 * Place
 */
void SurgemeServer::place(Pose target, Pose approach_pose,
                          double target_diameter,
                          std::vector<Pose> waypoints, InterpolationMethod interp_method,
                          double speed_cartesian)
{
  // Helper variables
  bool done = false;
  std::string stage = "";


  irob_msgs::InstrumentJawPart jaw_part =
      findInstrumentJawPartForSurgeme(irob_msgs::SurgemeGoal::PLACE);

  SurgemeSetting gs = calcSurgemeSetting(irob_msgs::SurgemeGoal::PLACE,
                                         jaw_part, target.orientation,
                                         target_diameter);

  for (Pose& p: waypoints)
    p += gs.t;

  // Start action

  // Navigate
  stage = "navigate";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(approach_pose + gs.t,
               speed_cartesian, waypoints, interp_method);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Approach
  stage = "approach";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(target + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, true);
  else
    return;

}

/**
 * Push
 */
void SurgemeServer::push(Pose target, Pose approach_pose, 
                         Eigen::Vector3d displacement,
                         std::vector<Pose> waypoints, InterpolationMethod interp_method,
                         double speed_cartesian, double speed_jaw)
{
  // Helper variables
  bool done = false;
  std::string stage = "";

  irob_msgs::InstrumentJawPart jaw_part =
      findInstrumentJawPartForSurgeme(irob_msgs::SurgemeGoal::PUSH);

  SurgemeSetting gs = calcSurgemeSetting(irob_msgs::SurgemeGoal::PUSH,
                                         jaw_part, target.orientation,
                                         0.0);

  // Start action

  // Navigate
  stage = "navigate";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(approach_pose + gs.t,
               speed_cartesian, waypoints, interp_method);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Close tool
  stage = "close_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_closed_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Approach
  stage = "approach";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(target + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Push
  stage = "push";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  Pose pushed_pose = arm.getPoseCurrent() + displacement;
  arm.moveTool(pushed_pose + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, true);
  else
    return;
}

/**
 * Dissect
 */
void SurgemeServer::dissect(Pose target, Pose approach_pose, 
                            Eigen::Vector3d displacement,
                            double target_diameter,
                            std::vector<Pose> waypoints, InterpolationMethod interp_method,
                            double speed_cartesian, double speed_jaw)
{
  // Helper variables
  bool done = false;
  std::string stage = "";

  irob_msgs::InstrumentJawPart jaw_part =
      findInstrumentJawPartForSurgeme(irob_msgs::SurgemeGoal::DISSECT);

  SurgemeSetting gs = calcSurgemeSetting(irob_msgs::SurgemeGoal::DISSECT,
                                         jaw_part, target.orientation,
                                         target_diameter);

  // Start action

  // Navigate
  stage = "navigate";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(approach_pose + gs.t,
               speed_cartesian, waypoints, interp_method);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Close tool
  stage = "close_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_closed_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Approach
  stage = "approach";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveTool(target + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Penetrate
  stage = "penetrate";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  Pose pushed_pose = arm.getPoseCurrent() + displacement;
  arm.moveTool(pushed_pose + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Open tool
  stage = "open_tool";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  arm.moveJaws(gs.jaw_open_angle, speed_jaw);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, false);
  else
    return;

  // Pull
  stage = "pull";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  pushed_pose = arm.getPoseCurrent() - displacement;
  arm.moveTool(pushed_pose + gs.t, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, true);
  else
    return;
}

/**
 * Maipulate
 */
// TODO rotation?
void SurgemeServer::manipulate(Eigen::Vector3d displacement,
                               double speed_cartesian)
{
  // Helper variables
  bool done = false;
  std::string stage = "";

  // Start action

  // Manipulate
  stage = "manipulate";
  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage);
  Pose manipulated_pose = arm.getPoseCurrent() + displacement;
  arm.moveTool(manipulated_pose, speed_cartesian);

  done = waitForActionDone(stage);
  if (done)
    handleActionState(stage, true);
  else
    return;

}  


// Simple relay
Pose SurgemeServer::getPoseCurrent()
{
  return arm.getPoseCurrent();
}

std::string SurgemeServer::getArmName()
{
  return arm.getName();
}

std::ostream& operator<<(std::ostream& os, 
                         const SurgemeServer::SurgemeSetting& d)
{
  return os << d.jaw_open_angle <<"\t" << d.jaw_open_angle <<"\t"
            << d.t;
}

}

using namespace ias;


/**
 * Surgeme server main
 */
int main(int argc, char **argv)
{

  // Initialize ros node
  ros::init(argc, argv, "surgeme_server");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string arm_name;
  priv_nh.getParam("arm_name", arm_name);

  double rate_command;
  priv_nh.getParam("rate", rate_command);


  // StartSurgeme server
  try {
    SurgemeServer surgeme(nh, arm_name, 1.0/rate_command);

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


