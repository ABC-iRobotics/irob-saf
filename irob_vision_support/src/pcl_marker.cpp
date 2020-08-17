#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <irob_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <irob_msgs/Point2D.h>
# include <irob_msgs/MarkerArray.h>





ros::Publisher marker_pos_pub;

bool marker_l_rcved = false;

irob_msgs::Point2D marker_l;

// This is to save on typing
typedef pcl::PointXYZ PointT;


void markers_l_cb (const irob_msgs::MarkerArray& msg)
{


    if(msg.markers.size()>0) {
        double avg_x = 0;
        double avg_y = 0;

        for (int i = 0; i < 4; i++)
        {
            avg_x += msg.markers[0].corners[i].x;
            avg_y += msg.markers[0].corners[i].y;
        }

        marker_l.x = std::round(avg_x / 4.0);
        marker_l.y = std::round(avg_y / 4.0);
        //ROS_INFO_STREAM("Marker: "<<marker_l);
        marker_l_rcved = true;
    }
}


void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
{
  if (marker_l_rcved) {
    // See http://wiki.ros.org/hydro/Migration for the source of this magic.
    pcl::PCLPointCloud2 pcl_pc;               // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(ros_pc, pcl_pc);

    // Convert point cloud to PCL native point cloud
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    PointT marker_pos = cloud->at(marker_l.x, marker_l.y);
    //PointT marker_pos = cloud->at(466, 336);
    //ROS_INFO_STREAM("W: " << cloud->width);
    //ROS_INFO_STREAM("H: " << cloud->height);
    //ROS_INFO_STREAM("marker_pos: " << marker_pos);
    if (!(std::isnan(marker_pos._PointXYZ::x) || std::isnan(marker_pos._PointXYZ::y) || std::isnan(marker_pos._PointXYZ::z))) {
        geometry_msgs::Point p;
        p.x = marker_pos._PointXYZ::x;
        p.y = marker_pos._PointXYZ::y;
        p.z = marker_pos._PointXYZ::z;
        // Publish the data
        marker_pos_pub.publish(p);
    }
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_marker");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber pcl_sub = nh.subscribe("/saf/stereo/points2", 1, cloud_cb);
  ros::Subscriber markers_l_sub = nh.subscribe("/saf/vision/markers", 1, markers_l_cb);
  // Create a ROS publisher for the output point cloud
  marker_pos_pub = nh.advertise<geometry_msgs::Point>("/saf/vision/target", 1);
  // Spin
  ros::spin ();
}
