/*
 *  robot_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-20
 *
 */
/*
 *  robot_client.cpp
 *
 *	Author(s): Tamas D. Nagy
 *	Created on: 2017-07-20
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <irob_general_robot/robot_client.hpp>
#include <irob_msgs/msg/tool_pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <irob_msgs/msg/instrument_info.hpp>
#include <actionlib/client/simple_action_client.hpp>

namespace saf {

RobotClient::RobotClient(rclcpp::Node::SharedPtr nh, std::string arm_name, double dt) :
  nh(nh), arm_name(arm_name), dt(dt),
  ac("robot/"+arm_name+"/robot_action", true)
{
  // Subscribe and advertise topics
  subscribeTopics();
  advertiseTopics();
  waitForActionServer();
}

RobotClient::~RobotClient()
{
  // TODO Auto-generated destructor stub
}

/*
 * Callbacks
 */

// Read pos and forward
void RobotClient::positionCartesianCurrentCB(
    const irob_msgs::msg::ToolPoseStamped::SharedPtr msg)
{
  position_cartesian_current = *msg;
  position_cartesian_current_pub->publish(msg);
}

// Read joints and forward
void RobotClient::jointStateCurrentCB(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_state_current = *msg;
  joint_state_current_pub->publish(msg);
}

void RobotClient::instrumentInfoCB(
    const irob_msgs::msg::InstrumentInfo::SharedPtr msg)
{
  instrument_info = *msg;
  instrument_info_pub->publish(msg);
}

void RobotClient::subscribeTopics()
{
  position_cartesian_current_sub =
      nh->create_subscription<irob_msgs::msg::ToolPoseStamped>(
        "robot/"+arm_name+"/position_cartesian_current_cf",
        10, std::bind(&RobotClient::positionCartesianCurrentCB, this, std::placeholders::_1));

  joint_state_current_sub =
      nh->create_subscription<sensor_msgs::msg::JointState>(
        "robot/"+arm_name+"/joint_state_current",
        10, std::bind(&RobotClient::jointStateCurrentCB, this, std::placeholders::_1));

  instrument_info_sub =
      nh->create_subscription<irob_msgs::msg::InstrumentInfo>(
        "robot/"+arm_name+"/instrument_info",
        10, std::bind(&RobotClient::instrumentInfoCB, this, std::placeholders::_1));
}

void RobotClient::advertiseTopics()
{
  position_cartesian_current_pub =
      nh->create_publisher<irob_msgs::msg::ToolPoseStamped>(
        "surgeme/"+arm_name+"/position_cartesian_current_cf", 10);

  joint_state_current_pub =
      nh->create_publisher<sensor_msgs::msg::JointState>(
        "surgeme/"+arm_name+"/joint_state_current", 10);

  instrument_info_pub =
      nh->create_publisher<irob_msgs::msg::InstrumentInfo>(
        "surgeme/"+arm_name+"/instrument_info", 10);
}

void RobotClient::waitForActionServer()
{
  RCLCPP_INFO(nh->get_logger(), "Waiting for action server...");
  ac.waitForServer();
  RCLCPP_INFO(nh->get_logger(), "Action server started");
}

ToolPose RobotClient::getPoseCurrent()
{
  while (position_cartesian_current.header.seq == 0)
  {
    rclcpp::spin_some(nh);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
  ToolPose ret(position_cartesian_current);
  return ret;
}

sensor_msgs::msg::JointState RobotClient::getJointStateCurrent()
{
  while (joint_state_current.header.seq == 0)
  {
    rclcpp::spin_some(nh);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
  sensor_msgs::msg::JointState ret(joint_state_current);
  return ret;
}

irob_msgs::msg::InstrumentInfo RobotClient::getInstrumentInfo()
{
  while (instrument_info.name.empty())
  {
    rclcpp::spin_some(nh);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }
  irob_msgs::msg::InstrumentInfo ret(instrument_info);
  return ret;
}

std::string RobotClient::getName()
{
  return arm_name;
}

/*
 *  Robot motions
 */

/**
 * Reset position of the instrument.
 *
 */
void RobotClient::resetPose(bool move_allowed)
{
  // Send a goal to the action
  irob_msgs::msg::RobotGoal goal;
  goal.action = irob_msgs::msg::RobotGoal::RESET_POSE;

  goal.move_allowed = move_allowed;
  ac.sendGoal(goal);
}

/**
 * Stop the robot.
 *
 */
void RobotClient::stop()
{
  // Send a goal to the action
  irob_msgs::msg::RobotGoal goal;
  goal.action = irob_msgs::msg::RobotGoal::STOP;

  goal.move_allowed = false;
  ac.sendGoal(goal);
}

/**
 * Move tool grippers.
 *
 * @param angle angle of jaws in deg
 * @param speed opening speed in deg/s
 */
void RobotClient::moveJaws(double angle, double speed)
{
  ToolPose p1 = getPoseCurrent();

  double angle_rad = degToRad(std::abs(angle));
  double speed_rad = degToRad(std::abs(speed));

  ToolPose p2 = p1;
  p2.jaw = angle_rad;

  Trajectory<ToolPose> tr = TrajectoryFactory::
      linearTrajectoryWithSmoothAcceleration(
        p1,
        p2,
        speed_rad, speed_rad * 10.0, dt);

  irob_msgs::msg::RobotGoal goal;
  goal.action = irob_msgs::msg::RobotGoal::FOLLOW_TRAJECTORY;
  goal.move_allowed = true;

  tr.copyToRosTrajectory(goal.trajectory);

  ac.sendGoal(goal);

  // Not waiting for action finish here, a notification will be received
  // in actionDoneCB
}

/**
 * Move tool, gripper stays fixed.
 *
 * @param target target position
 * @param speed speed of the motion in mm/s
 * @param waypoints move through a vector of waypoints
 * @param interp_method method used to interpolate between positions
 */
void RobotClient::moveTool(ToolPose target, double speed,
                           std::vector<ToolPose> waypoints /* = empty vector */,
                           InterpolationMethod interp_method /* = LINEAR */)
{
  ToolPose p1 = getPoseCurrent();
  Trajectory<ToolPose> tr;

  // Jaw cannot be changed!
  target.jaw = p1.jaw;
  for (ToolPose &p : waypoints)
    p.jaw = p1.jaw;

  if (waypoints.empty()) {
    // Go straight to target

    tr = TrajectoryFactory::
        linearTrajectoryWithSmoothAcceleration(
          p1,
          target,
          speed, speed * 10.0, dt);

  } else if (interp_method == LINEAR) {
    // Linear trajectory through waypoints
    tr = TrajectoryFactory::
        linearTrajectoryWithSmoothAcceleration(
          p1,
          waypoints,
          target,
          speed, speed * 10.0, dt);
  } else {
    // Go on Bezier curve through waypoints
    throw std::runtime_error("Trajectory through waypoints is not implemented yet");
  }

  irob_msgs::msg::RobotGoal goal;
  goal.action = irob_msgs::msg::RobotGoal::FOLLOW_TRAJECTORY;
  goal.move_allowed = true;
  tr.copyToRosTrajectory(goal.trajectory);

  ac.sendGoal(goal);
}

/**
 * Move tool, gripper stays fixed.
 *
 * @param target target position
 * @param speed speed of the motion in mm/s
 * @param waypoints move through a vector of waypoints
 * @param interp_method method used to interpolate between positions
 */
void RobotClient::moveJoints(sensor_msgs::msg::JointState joint_state)
{
  irob_msgs::msg::RobotGoal goal;
  goal.action = irob_msgs::msg::RobotGoal::MOVE_JOINT;
  goal.move_allowed = true;
  goal.joint_state = joint_state;
  ac.sendGoal(goal);
}

/**
 * Is the action done.
 * @param spin do a rclcpp::spin_some() before query
 * @return true if the action is done
 */
bool RobotClient::isActionDone(bool spin /* = true */)
{
  return ac.isDone(spin);
}

/**
 * @return state of the action client
 */
actionlib::SimpleClientGoalState RobotClient::getState()
{
  return ac.getState();
}

/**
 * @param spin do a rclcpp::spin_some() before query
 * @return feedback
 */
irob_msgs::msg::RobotFeedback RobotClient::getFeedback(bool spin /* = true */)
{
  return ac.getFeedback(spin);
}

/**
 * @param spin do a rclcpp::spin_some() before query
 * @return  result
 */
irob_msgs::msg::RobotResult RobotClient::getResult(bool spin /* = true */)
{
  return ac.getResult(spin);
}

}
