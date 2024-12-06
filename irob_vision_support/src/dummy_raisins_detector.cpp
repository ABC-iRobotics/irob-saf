#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include <irob_utils/tool_pose.hpp>
#include <irob_utils/utils.hpp>
#include <irob_msgs/GraspObject.h>
#include <irob_msgs/Environment.h>
#include <irob_utils/abstract_directions.hpp>

ros::Publisher env_pub;

geometry_msgs::Pose eigenAffine3dToPose(const Eigen::Affine3d &transform)
{
    geometry_msgs::Pose pose;

    pose.position.x = transform.translation().x() * 1000;
    pose.position.y = transform.translation().y() * 1000;
    pose.position.z = transform.translation().z() * 1000;

    Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

void chatterCallback(const visualization_msgs::MarkerArrayPtr &msg)
{
    irob_msgs::Environment env_msg;
    env_msg.valid = irob_msgs::Environment::VALID;
    env_msg.header.stamp = ros::Time::now();
    env_msg.header.frame_id = "camera";

    int i{0};
    for (const auto &marker : msg->markers)
    {
        irob_msgs::GraspObject obj_msg;
        Eigen::Translation3d t(0, 0, -0.01);
        Eigen::Affine3d markerPose(saf::unwrapMsg<geometry_msgs::Pose, Eigen::Affine3d>(marker.pose));
        Eigen::Affine3d approach_pose(t.inverse() * markerPose);
        obj_msg.approach_poses.push_back(eigenAffine3dToPose(approach_pose));

        Eigen::Affine3d grasp_pose(markerPose);
        obj_msg.grasp_poses.push_back(eigenAffine3dToPose(grasp_pose));

        obj_msg.id = i;
        std::ostringstream oss;
        oss << "block#" << i;
        obj_msg.name = oss.str();

        env_msg.objects.push_back(obj_msg);
        i++;
    }

    env_pub.publish(env_msg);
    ros::Duration(1.0).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_raisins_detector");
    ros::NodeHandle n;

    try
    {
        env_pub = n.advertise<irob_msgs::Environment>("blocks_grasp", 10);
        ros::Subscriber sub = n.subscribe("/saf/dummy_raisins_target", 1000, chatterCallback);

        ros::spin();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    ros::shutdown();
    return 0;
}