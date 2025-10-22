/*
 * 	robot_server_psm.hpp
 *
 *	Author(s): Tamas Levendovics
 *	Created on: 2016-11-07
 *
 *  Robot server for dVRK PSMs.
 *
 */

#ifndef ROBOT_SERVER_PSM_HPP_
#define ROBOT_SERVER_PSM_HPP_

#include <iostream>
#include <sstream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <saf_utils/utils.hpp>
#include <saf_dvrk/robot_server_dvrk.hpp>
#include <saf_dvrk/arm_types.hpp>
#include <saf_utils/topic_name_loader.hpp>
#include <saf_utils/tool_pose.hpp>
#include <saf_utils/trajectory.hpp>

namespace saf {

    class RobotServerPSM: public RobotServerDVRK {

    private:

        // Publishers
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_jaw_pub;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_jaw_sub;

        // States
        sensor_msgs::msg::JointState jaw_measured_js;

        void advertiseLowLevelTopics();
        void subscribeLowLevelTopics();

    public:
        RobotServerPSM(rclcpp::Node::SharedPtr node, ArmTypes arm_type, std::string name, bool use_sim);
        ~RobotServerPSM();

        void resetPose(bool);

        void measured_cp_cb(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

        void positionJawCurrentCB(const sensor_msgs::msg::JointState::SharedPtr msg);

        ToolPose getPoseCurrent();

        void moveCartesianAbsolute(ToolPose pose, double velocity = 0.01);
        void moveJawRelative(double position, double velocity = 0.01);
        void moveJawAbsolute(double position, double velocity = 0.01);
    };

}  // namespace saf

#endif /* ROBOT_SERVER_PSM_HPP_ */
