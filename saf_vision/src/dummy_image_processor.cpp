/*
*  dummy_image_processor.cpp
 *
 *  Author(s): Tamas Levendovics
 *  Created on: 2016-10-26
 *  ROS 2 port: 2025-10-14
 *
 */

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>


/**
 * A trivial image "processor" that fabricates a moving target transform.
 * Replace this with your real model-based or ML-based vision implementation.
 */
namespace saf {

    class DummyImageProcessor : public rclcpp::Node {
    public:
        DummyImageProcessor()
        : rclcpp::Node("dummy_image_processor")
            {
                marker_sub_= this->create_subscription<visualization_msgs::msg::Marker>(
                                "marker", rclcpp::SensorDataQoS(),
                                std::bind(&DummyImageProcessor::marker_cb_, this, std::placeholders::_1));

				result_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("result", 10);
                RCLCPP_INFO(get_logger(), "dummy_image_processor running");
            }


        geometry_msgs::msg::TransformStamped processData(const visualization_msgs::msg::Marker::SharedPtr marker) const
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = marker->header.stamp;
            t.header.frame_id = marker->header.frame_id;


            t.transform.translation.x = marker->pose.position.x * 1000.0; // millimeters
            t.transform.translation.y = marker->pose.position.y * 1000.0;
            t.transform.translation.z = marker->pose.position.z * 1000.0;
            t.transform.rotation.x = marker->pose.orientation.x;
            t.transform.rotation.y = marker->pose.orientation.y;
            t.transform.rotation.z = marker->pose.orientation.z;
            t.transform.rotation.w = marker->pose.orientation.w;
            return t;
        }


        void marker_cb_(const visualization_msgs::msg::Marker::SharedPtr msg)
        {
            geometry_msgs::msg::TransformStamped t = processData(msg);
			result_pub_->publish(t);
        }

	private:
        rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
    	rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr result_pub_;

    };
} //

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<saf::DummyImageProcessor>());
    rclcpp::shutdown();
    return 0;
}


