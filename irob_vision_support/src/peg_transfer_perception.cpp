#include <irob_vision_support/peg_transfer_perception.hpp>

using namespace saf;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
        ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;



pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}


PegTransferPerception::PegTransferPerception(ros::NodeHandle nh,
                                             std::string ply_filename,
                                             std::string configfile):
  nh(nh), ply_filename(ply_filename), configfile(configfile)
{

  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("point_cloud", 1);
  obj_pub = nh.advertise<sensor_msgs::PointCloud2> ("object", 1);


}



void PegTransferPerception::runPerception()
{
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Start streaming with default recommended configuration
  pipe.start();

  DP object(DP::load(ply_filename));
  ROS_INFO_STREAM("object read: ");

  // Wait for the next set of frames from the camera
  int n = 0;
  while(ros::ok())
  {
    auto frames = pipe.wait_for_frames();
    ROS_INFO_STREAM("Pointcloud recieved.");

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    auto pcl_points = points_to_pcl(points);

    pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_points);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);

    std::vector<pcl_ptr> layers;
    layers.push_back(pcl_points);
    layers.push_back(cloud_filtered);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_filtered, msg);
    msg.header.frame_id = "base";
    pcl_pub.publish(msg);


    if (n == 30) {
    n = -1;
    // ICP
    ROS_INFO_STREAM("before icp");
    sensor_msgs::PointCloud2 scene_Cloud_libpointmatcher;
    PointMatcher<float>::DataPoints scene =
        PointMatcher_ros::rosMsgToPointMatcherCloud<float>(
                          msg, false);

    PointMatcher<float>::ICP icp;

    std::ifstream ifs(configfile.c_str());
    if (!ifs.good())
    {
      ROS_INFO_STREAM("Cannot open config file ");
    }
    icp.loadFromYaml(ifs);

    //icp.setDefault();
    PointMatcher<float>::TransformationParameters T = icp(object, scene);

    std::cout << "Transformation Matrix = \n" << T << std::endl;
    PointMatcher<float>::DataPoints transformed_object(object);
    icp.transformations.apply(transformed_object, T);

    sensor_msgs::PointCloud2 transformed_pcd = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(transformed_object, "base", ros::Time::now());

    ROS_INFO_STREAM("after icp");
    obj_pub.publish(transformed_pcd);

    }
    ros::Duration(0.01).sleep();
    n++;


  }
  pipe.stop();

}



int main(int argc, char * argv[]) try
{

  // Initialize ROS
  ros::init (argc, argv, "peg_transfer_perception");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string ply_filename;
  priv_nh.getParam("ply_filename", ply_filename);

  std::string configfile;
  priv_nh.getParam("configfile", configfile);

  PegTransferPerception ptp(nh, ply_filename, configfile);
  ptp.runPerception();

  return 1;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

