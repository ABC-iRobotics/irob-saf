#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <cmath>

#include <irob_utils/utils.hpp>
#include <irob_utils/trajectory.hpp>
#include <irob_utils/trajectory_factory.hpp>

using namespace saf;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_raisins_target");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    double rate;
    priv_nh.getParam("rate", rate);

    std::string frame_id;
    priv_nh.getParam("frame_id", frame_id);

    double amplitude;
    priv_nh.getParam("amplitude", amplitude);

    std::vector<double> centrum;
    priv_nh.getParam("centrum", centrum);

    double T;
    priv_nh.getParam("T", T);

    ros::Publisher vis_pub =
        nh.advertise<visualization_msgs::MarkerArray>("dummy_raisins_target", 0);

    ros::Rate loop_rate(rate);

    try
    {
        int i = 0;
        int j = 0;

        Trajectory<Eigen::Vector3d> tr = TrajectoryFactory::circleTrajectoryHorizontal(
            Eigen::Vector3d(centrum[0] + amplitude, centrum[1], centrum[2]),
            2.0 * M_PI, Eigen::Vector3d(centrum[0], centrum[1], centrum[2]),
            T, 1 / rate);

        while (ros::ok())
        {

            visualization_msgs::MarkerArray markerArray;

            for (size_t i = 0; i < 5; i++)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = frame_id;
                marker.header.stamp = ros::Time();
                marker.ns = "dvrk_viz";
                marker.id = i;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position = wrapToMsg<geometry_msgs::Point, Eigen::Vector3d>(tr[j % tr.size()]);
                marker.pose.position.x += (i * 0.02);
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.005;
                marker.scale.y = 0.005;
                marker.scale.z = 0.005;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                markerArray.markers.push_back(marker);
            }

            vis_pub.publish(markerArray);

            loop_rate.sleep();

            j++;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }

    std::cout << std::endl
              << "Program finished succesfully, shutting down ..." << std::endl;

    ros::shutdown();
    return 0;
}