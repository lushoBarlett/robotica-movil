#ifndef MAPPING_NODE_HPP
#define MAPPING_NODE_HPP

#include "cam_params.hpp"
#include "node_utils.hpp"
#include "poses.hpp"
#include "stereoproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class MappingNode : public rclcpp::Node {
  public:
    MappingNode(const std::string &bag_path, const std::string &poses_csv, bool dense,
                bool estimate_displacement);
    void process_bag();

  private:
    // Member variables
    bool dense_, estimate_displacement_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::vector<Pose> poses_;
    double base_line_btw_cams_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    // Constants
    const double tolerance_ = 0.05;

    // Member functions
    void initialize_rosbag_reader(const std::string &bag_path);
    bool process_bag_message(rclcpp::Serialization<sensor_msgs::msg::Image> &image_serializer,
                             rclcpp::Time &timestamp_cam0,
                             std::shared_ptr<sensor_msgs::msg::Image> &image_msg_cam0,
                             rclcpp::Time &timestamp_cam1,
                             std::shared_ptr<sensor_msgs::msg::Image> &image_msg_cam1);

    void process_images(cv::Mat &image_cam0, cv::Mat &image_cam1, rclcpp::Time timestamp_cam0,
                        rclcpp::Time timestamp_cam1);
};

#endif // MAPPING_NODE_HPP
