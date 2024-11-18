#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <signal.h>
#include <vector>
#include <iostream>
#include <string>
#include <memory>
#include "stereoproc.hpp"
#include "bagreader.hpp"
#include "poses.hpp"
#include "cam_params.hpp"


std::atomic<bool> block(false); // Flag to control pause state

void handleSignal(int signal) {
    if (signal == SIGTSTP) {
        block = !block;
    }
}

static sensor_msgs::msg::PointCloud2 points3DtoCloudMsg(const std::vector<cv::Point3d>& points3D, const std::string &frame_id) {
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
    cloud_msg.header.frame_id = frame_id;

    return cloud_msg;
}

void publish_camera_pose(std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster, const std::shared_ptr<tf2_ros::StaticTransformBroadcaster>& static_broadcaster, const std::string &child_frame_id, const std::string &frame_id, Pose pose) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set header and frame information
    //transform_stamped.header.stamp = pose.timestamp;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;

    // Set translation
    transform_stamped.transform.translation.x = pose.x;
    transform_stamped.transform.translation.y = pose.y;
    transform_stamped.transform.translation.z = pose.z;

    transform_stamped.transform.rotation.x = pose.qx;
    transform_stamped.transform.rotation.y = pose.qy;
    transform_stamped.transform.rotation.z = pose.qz;
    transform_stamped.transform.rotation.w = pose.qw;

    // Publish the transform
    if (tf_broadcaster)
        tf_broadcaster->sendTransform(transform_stamped);
    else if (static_broadcaster)
        static_broadcaster->sendTransform(transform_stamped);
}

void map_rosbag_cam_poses(int argc, char **argv, bool dense)
{
    std::signal(SIGTSTP, handleSignal);

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("rosbag_reader");
    auto tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

    // Create a reader object for the ROS bag
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());

    // Set storage options and path to your ROS bag file
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "../data/ros2.bag2";
    storage_options.storage_id = "sqlite3";

    // Open the ROS bag file
    reader.open(storage_options);

    // Read ground truth poses of the camera's body
    std::string csv_file = "../data/ground_truth.csv";
    std::vector<Pose> poses = readGroundTruthCSV(csv_file);
    double tolerance = 0.1;  // Set tolerance

    // Initialize variables to store rosbag cameras images
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_cam0 = nullptr;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_cam1 = nullptr;
    rclcpp::Time timestamp_cam0;
    rclcpp::Time timestamp_cam1;

    // Get poses of the cameras
    Pose left_cam_pose_wrt_body = get_left_cam_transform_wrt_body();
    Pose right_cam_pose_wrt_body = get_right_cam_transform_wrt_body();
    float base_line_btw_cams = get_base_line_btw_cams();
    publish_camera_pose(nullptr, static_broadcaster, "left_cam", "body", left_cam_pose_wrt_body);
    publish_camera_pose(nullptr, static_broadcaster, "right_cam", "body", right_cam_pose_wrt_body);

    // Loop over the messages in the bag file
    while (rclcpp::ok() && reader.has_next()) {
        // Read the next serialized message
        auto bag_message = reader.read_next();

        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

        // Check the topic and type to handle specific message types
        if (bag_message->topic_name == "/cam0/image_raw") {
            image_msg_cam0 = std::make_shared<sensor_msgs::msg::Image>();
            image_serializer.deserialize_message(&serialized_msg, image_msg_cam0.get());
            timestamp_cam0 = rclcpp::Time(image_msg_cam0->header.stamp);
        }
        if (bag_message->topic_name == "/cam1/image_raw") {
            image_msg_cam1 = std::make_shared<sensor_msgs::msg::Image>();
            image_serializer.deserialize_message(&serialized_msg, image_msg_cam1.get());
            timestamp_cam1 = rclcpp::Time(image_msg_cam1->header.stamp);
        }

        // If an image from each camera has been read
        if (image_msg_cam0 && image_msg_cam1) {
            // Check if the timestamps match
            if (timestamp_cam0 == timestamp_cam1) {
                // The images are synchronized, proceed with triangulation
                RCLCPP_INFO(node->get_logger(), "Images are synchronized: %f", timestamp_cam0.seconds());

                // Convert sensor_msgs::Image to cv::Mat using cv_bridge
                cv::Mat image_cam0 = cv_bridge::toCvCopy(image_msg_cam0, sensor_msgs::image_encodings::BGR8)->image;
                cv::Mat image_cam1 = cv_bridge::toCvCopy(image_msg_cam1, sensor_msgs::image_encodings::BGR8)->image;

                Pose closest_pose;
                bool is_close_pose = findClosestPose(poses, timestamp_cam0.seconds(), tolerance, &closest_pose);

                if (!is_close_pose) {
                    std::cout << "Couldn't find close pose in ground truth." << std::endl;
                    image_msg_cam0.reset();
                    image_msg_cam1.reset();
                    continue;
                }

                publish_camera_pose(tf_broadcaster_, nullptr, "body", "map", closest_pose);

                cv::Mat R_estimated, T_estimated;
                std::vector<cv::Point3d> points3D;

                if (dense)
                    process_images_dense(image_cam0, image_cam1, &points3D, block);
                else
                    process_images(image_cam0, image_cam1, &R_estimated, &T_estimated, &points3D, block);

                if (!R_estimated.empty() && !T_estimated.empty()) {
                    T_estimated *= base_line_btw_cams;
                    Pose right_cam_pose_wrt_left_cam = createPose(R_estimated, T_estimated);
                    publish_camera_pose(tf_broadcaster_, nullptr, "right_cam_est", "left_cam", right_cam_pose_wrt_left_cam);
                }

                sensor_msgs::msg::PointCloud2 cloud_msg = points3DtoCloudMsg(points3D, "left_cam");

                publisher->publish(cloud_msg);
            } else {
                // If timestamps do not match, log the mismatch
                RCLCPP_INFO(node->get_logger(), "Timestamp mismatch: cam0 = %f, cam1 = %f", timestamp_cam0.seconds(), timestamp_cam1.seconds());
            }

            // Reset the images to look for the next pair
            image_msg_cam0.reset();
            image_msg_cam1.reset();
        }
    }

    // Clean up and shutdown
    rclcpp::shutdown();
    return;
}
