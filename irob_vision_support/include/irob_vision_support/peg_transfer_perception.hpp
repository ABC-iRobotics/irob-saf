#ifndef PEG_TRANSFER_PERCEPTION_H
#define PEG_TRANSFER_PERCEPTION_H


#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>

#include "irob_utils/tool_pose.hpp"
#include "irob_utils/utils.hpp"
#include "irob_utils/abstract_directions.hpp"

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

#include "pointmatcher/PointMatcher.h"

#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "pointmatcher_ros/get_params_from_server.h"
#include "pointmatcher_ros/ros_logger.h"

namespace saf {

class PegTransferPerception
{
private:
  ros::NodeHandle nh;
  ros::Publisher pcl_pub;
  ros::Publisher obj_pub;
  std::string ply_filename;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);


public:
  PegTransferPerception(ros::NodeHandle, std::string);
  void runPerception();

};

}

#endif // PEG_TRANSFER_PERCEPTION_H
