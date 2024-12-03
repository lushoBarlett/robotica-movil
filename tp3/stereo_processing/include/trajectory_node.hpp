#ifndef TRAJECTORY_NODE_HPP
#define TRAJECTORY_NODE_HPP

#include "cam_params.hpp"
#include "node_utils.hpp"
#include "poses.hpp"
#include "stereoproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <signal.h>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

class TrajectoryNode : public rclcpp::Node {
  public:
    TrajectoryNode(const std::string &bag_path, const std::string &ground_truth_path,
                   const std::string &cam_filename, const std::string &body_filename);
    void process_bag();

  private:
    void initialize_rosbag_reader(const std::string &bag_path);

    bool process_bag_message(rclcpp::Serialization<sensor_msgs::msg::Image> &image_serializer,
                             rclcpp::Time &timestamp_image_0,
                             std::shared_ptr<sensor_msgs::msg::Image> &image_msg_0,
                             rclcpp::Time &timestamp_image_1,
                             std::shared_ptr<sensor_msgs::msg::Image> &image_msg_1);
    void process_images_and_update_pose(cv::Mat &image_0, cv::Mat &image_1,
                                        rclcpp::Time timestamp_image_1);

    std::ofstream cam_file, body_file;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::vector<Pose> poses_;
    Pose cam_pose_wrt_body_;
    Pose cam_pose_wrt_map_;
    Pose body_pose_wrt_map_;
    const double tolerance_ = 0.05;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer_;
};

#endif // TRAJECTORY_NODE_HPP
