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
using pcl_rgb_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;


std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}


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

pcl_rgb_ptr points_to_rgb_pcl(const rs2::points& points, const rs2::video_frame& RGB)
{
    pcl_rgb_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width  = static_cast<uint32_t>( sp.width()  );
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(RGB, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].b = std::get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].r = std::get<0>(RGB_Color); // Reference tuple<0>r
    }

   return cloud; // PCL RGB Point Cloud generated
}



PegTransferPerception::PegTransferPerception(ros::NodeHandle nh,
                                             std::string ply_filename,
                                             std::string configfile,
                                             int hue_lower,
                                             int hue_upper,
                                             int saturation_lower,
                                             int saturation_upper):
  nh(nh), ply_filename(ply_filename), configfile(configfile),
  hue_lower(hue_lower), hue_upper(hue_upper), saturation_lower(saturation_lower),
  saturation_upper(saturation_upper)
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
    auto RGB = frames.get_color_frame();

    // Map Color texture to each point
    pc.map_to(RGB);

    // Generate Point Cloud
    auto points = pc.calculate(depth);

    // Convert generated Point Cloud to PCL Formatting
    pcl_rgb_ptr rgb_cloud = points_to_rgb_pcl(points, RGB);


    pcl_rgb_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
        hue_condition_1(new pcl::PackedHSIComparison<pcl::PointXYZRGB>(
                                            "h", pcl::ComparisonOps::GT,   hue_lower));

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(
                        new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
        hue_condition_2(new pcl::PackedHSIComparison<pcl::PointXYZRGB>(
                                            "h", pcl::ComparisonOps::LT,   hue_upper));

    pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
        saturation_condition_1(new pcl::PackedHSIComparison<pcl::PointXYZRGB>(
                                            "s", pcl::ComparisonOps::GT,
                                            saturation_lower));


    pcl::PackedHSIComparison<pcl::PointXYZRGB>::Ptr
        saturation_condition_2(new pcl::PackedHSIComparison<pcl::PointXYZRGB>(
                                            "s", pcl::ComparisonOps::LT,
                                            saturation_upper));

    color_cond->addComparison (hue_condition_1);
    color_cond->addComparison (hue_condition_2);
    color_cond->addComparison (saturation_condition_1);
    color_cond->addComparison (saturation_condition_2);

    // Build the filter
    color_filter.setInputCloud(rgb_cloud);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_filtered);

    std::vector<pcl_rgb_ptr> layers;
    layers.push_back(rgb_cloud);
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

  int hue_lower;
  priv_nh.getParam("hue_lower", hue_lower);

  int hue_upper;
  priv_nh.getParam("hue_upper", hue_upper);


  int saturation_lower;
  priv_nh.getParam("saturation_lower", saturation_lower);

  int saturation_upper;
  priv_nh.getParam("saturation_upper", saturation_upper);

  PegTransferPerception ptp(nh, ply_filename, configfile, hue_lower, hue_upper, saturation_lower, saturation_upper);
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

