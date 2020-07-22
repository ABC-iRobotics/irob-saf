/*
 *  surgeme_server.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-11-06
 *
 */

#include <irob_motion/surgeme_server.hpp>

namespace saf {


const double SurgemeServer::DEFAULT_LOOP_RATE = 10.0;				// Hz


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

  Eigen::Vector3d displacement(unwrapMsg<geometry_msgs::Point,Eigen::Vector3d>(goal->displacement));
  Eigen::Vector3d marker(unwrapMsg<geometry_msgs::Point,Eigen::Vector3d>(goal->marker));
  Eigen::Vector3d desired(unwrapMsg<geometry_msgs::Point,Eigen::Vector3d>(goal->desired));
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

  // Call surgeme function
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

      // MOVE_CAM
    case irob_msgs::SurgemeGoal::MOVE_CAM:
    {
      move_cam(marker, desired, goal -> speed_cartesian);
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

/**
 * Check if the installed instrument is usable for
 * the current surgeme.
 * @param surgeme_type surgeme
 * @return is the surgeme can be executed
 */
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

  if (arm.getInstrumentInfo().basic_type == irob_msgs::InstrumentInfo::CAMERA
      && (	surgeme_type == irob_msgs::SurgemeGoal::MOVE_CAM))
    return true;

  if(findInstrumentJawPartForSurgeme(surgeme_type).type
     != irob_msgs::InstrumentJawPart::JOINT)
    return true;

  return false;
}

/**
 * Find the part of the instument's jaws usable for the surgeme.
 * @param surgeme_type surgeme
 * @return jaw part
 */
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

/**
 * Calculate parameters of the current surgeme based on
 * the target and the installed instrument.
 * @param surgeme_type surgeme
 * @param jaw_part the part of the jaw, used for the surgeme
 * @param target_ori desired tool orientation at target
 * @param target_diameter size of the target object
 * @param compression_rate rate of the desired compression of the target (used in grasp)
 * @return the calculated surgeme setting
 */
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
        ||  surgeme_type == irob_msgs::SurgemeGoal::RELEASE
        ||  surgeme_type == irob_msgs::SurgemeGoal::CUT)
  {

    double dist = jaw_part.end - (target_diameter / 2.0);

    g.t = -1.0 * dist * quatToVec<Eigen::Quaternion<double>,
        Eigen::Vector3d>(target_ori);
    //g.t *= 1.0;

    g.jaw_closed_angle = (2.0 * atan(((target_diameter / 2.0)
                                      * compression_rate) / dist))
        * (180.0 / M_PI);

    g.jaw_open_angle = (2.0 * atan(((target_diameter / 2.0)
                                    * 3.5) / dist))
        * (180.0 / M_PI);

    if (surgeme_type == irob_msgs::SurgemeGoal::CUT)
        g.jaw_closed_angle = 0.0;

    ROS_INFO_STREAM("g.t: " << g.t);
    return g;
  }

  /*if (surgeme_type == irob_msgs::SurgemeGoal::CUT)
  {
    double dist = jaw_part.end - (target_diameter / 2.0);

    g.t = -1.0 * dist * quatToVec<Eigen::Quaternion<double>,
        Eigen::Vector3d>(target_ori);

    g.jaw_open_angle = (2.0 * atan((target_diameter / 2.0) / dist))
        * (180.0 / M_PI);

    g.jaw_closed_angle = 0.0;
    return g;
  }*/

  // End of jaw
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
 * Stop the arm immediately.
 *
 * @brief SurgemeServer::stop
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
 * Navigate freely to the target position.
 *
 * @brief SurgemeServer::nav_to_pos
 * @param target desired position
 * @param waypoints waypoints to be touched during navigation
 * @param interp_method method for interpolation between positions
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
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
 * Grasp object.
 *
 * @brief SurgemeServer::grasp
 * @param target object position
 * @param approach_pose approach object from position with grippers open
 * @param target_diameter diameter of the object
 * @param compression_rate desired compression of the object
 * @param waypoints waypoints to be touched during navigation
 * @param interp_method method for interpolation between positions
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
 * @param speed_jaw jaw speed in deg/s
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
 * Cut object using scrissors.
 *
 * @brief SurgemeServer::cut
 * @param target object position
 * @param approach_pose approach object from position with grippers open
 * @param target_diameter diameter of the object
 * @param waypoints waypoints to be touched during navigation
 * @param interp_method method for interpolation between positions
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
 * @param speed_jaw jaw speed in deg/s
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
 * Release grasped object.
 *
 * @brief SurgemeServer::release
 * @param approach_pose positon to retract tool with open grippers
 * @param target_diameter diameter of the object
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
 * @param speed_jaw jaw speed in deg/s
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
 * Place grasped object to position, jaws angles remain constant.
 *
 * @brief SurgemeServer::place
 * @param target desired object position
 * @param approach_pose approach object from position
 * @param target_diameter diameter of the object
 * @param waypoints waypoints to be touched during navigation
 * @param interp_method method for interpolation between positions
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
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
 * Push tissue with tool tip.
 *
 * @brief SurgemeServer::push
 * @param target object position
 * @param approach_pose approach object from position
 * @param displacement displacement vector of the tool after reached target position
 * @param waypoints waypoints to be touched during navigation
 * @param interp_method method for interpolation between positions
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
 * @param speed_jaw jaw speed in deg/s
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
 * Dissect soft tissue using an opening motion.
 *
 * @brief SurgemeServer::dissect
 * @param target object position
 * @param approach_pose approach object from position
 * @param target_diameter for the open state of the grippers
 * @param waypoints waypoints to be touched during navigation
 * @param interp_method method for interpolation between positions
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
 * @param speed_jaw jaw speed in deg/s
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
 * Manipulate grasped soft tissue.
 *
 * @brief SurgemeServer::manipulate
 * @param displacement displacement vector of the tool after reached target position
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
 */
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

/**
 * Move the endoscopic camer so the marker_position_tcp appears to move to
 * desired_pos_tcp.
 *
 * @brief SurgemeServer::move_cam
 * @param marker_pos_tcp TODO
 * @param desired_pos_tcp TODO
 * @param speed_cartesian cartesian speed of the tool tip in mm/s
 */
void SurgemeServer::move_cam(Eigen::Vector3d marker_pos_cam,
                             Eigen::Vector3d desired_pos_cam,
                               double speed_cartesian)
{
  // Helper variables
  bool done = false;
  std::string stage = "";

  // Trasform positions from cam to base frame
  Pose p = arm.getPoseCurrent();
  Eigen::Transform<double,3,Eigen::Affine> T_cam_base(p.toTransform());
  Eigen::Vector3d m_base = T_cam_base.inverse() * marker_pos_cam;
  Eigen::Vector3d d_base = T_cam_base.inverse() * desired_pos_cam;
  ROS_INFO_STREAM("T_cam_base trans: " << T_cam_base.translation());
  ROS_INFO_STREAM("m_base: " << m_base);
  ROS_INFO_STREAM("d_base: " << d_base);



  // Conversion to spherical coordinates r, phi, theta
  double r_M = m_base.norm();
  double phi_M = atan2(m_base.y(), m_base.x());
  double theta_M = acos(m_base.z() / r_M);

  ROS_INFO_STREAM("theta_M: " << theta_M << std::endl <<
                  "phi_M: " << phi_M << std::endl <<
                  "r_M: " << r_M);

  double r_D = d_base.norm();
  double phi_D = atan2(d_base.y(), d_base.x());
  double theta_D = acos(d_base.z() / r_D);

  ROS_INFO_STREAM("theta_D: " << theta_D << std::endl <<
                  "phi_D: " << phi_D << std::endl <<
                  "r_D: " << r_D);

  // Calculate desired changes in agles and zoom
  // Aplha rotates around x axis, beta around y axis

  double alpha = -(theta_M - theta_D);
  double beta = -(0.0);//phi_M - phi_D;
  double delta_r = -(r_M - r_D);

  ROS_INFO_STREAM("alpha: " << alpha << std::endl <<
                  "beta: " << beta << std::endl <<
                  "zoom: " << delta_r);

  // Calculate rotation matrices
  Eigen::Quaternion<double> q_x;
  q_x = Eigen::AngleAxis<double>(alpha, Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> q_y;
  q_y = Eigen::AngleAxis<double>(beta, Eigen::Vector3d::UnitY());
  Eigen::Translation<double,3> Trans_zoom = Eigen::Translation<double,3>(0, 0, delta_r);

  Pose p_new =  (Trans_zoom * q_y * q_x) * p;


  // Start action

  // Move camera
  stage = "move_cam";

  ROS_INFO_STREAM(arm.getName()  << ": starting " << stage
                            << std::endl<<"pose: "<< p
                            << std::endl << "new pose: " << p_new
                            << std::endl << "jointstate: " << arm.getJointStateCurrent() << std::endl);
  arm.moveTool(p_new, speed_cartesian);

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

// Simple relay
sensor_msgs::JointState SurgemeServer::getJointStateCurrent()
{
  return arm.getJointStateCurrent();
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

using namespace saf;


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


  // Start surgeme server
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


