#include "publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

void publishContinuously(const std::vector<cv::Point3d>& points3D)
{
    double rate_hz = 10.0;
    // Initialize the ROS2 node (without rclcpp::spin)
    auto node = rclcpp::Node::make_shared("single_point_cloud_publisher");

    // Create a publisher for the PointCloud2 topic
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

    // Create a PointCloud2 message from the 3D points
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = points3D.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < points3D.size(); ++i) {
        cloud.points[i].x = points3D[i].x;
        cloud.points[i].y = points3D[i].y;
        cloud.points[i].z = points3D[i].z;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "base_link";

    rclcpp::Rate rate(rate_hz);  // Publish at the specified rate (in Hz)

    // Continuously publish the message
    while (rclcpp::ok()) {
        // Update the header timestamp
        cloud_msg.header.stamp = node->now();

        // Publish the message
        publisher->publish(cloud_msg);

        // Spin the node to handle any callbacks
        rclcpp::spin_some(node);

        // Sleep to maintain the loop rate
        rate.sleep();
    }

    rclcpp::shutdown();
}
