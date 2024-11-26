#include "bagreader.hpp"
#include "cam_params.hpp"
#include "poses.hpp"
#include "stereoproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <signal.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

std::atomic<bool> block(false); // Flag to control pause state

void handleSignal(int signal) {
    if (signal == SIGTSTP) {
        block = !block;
    }
}

static sensor_msgs::msg::PointCloud2 points3DtoCloudMsg(const std::vector<cv::Point3d> &points3D,
                                                        const std::string &frame_id) {
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

void publish_camera_pose(
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
    const std::shared_ptr<tf2_ros::StaticTransformBroadcaster> &static_broadcaster,
    const std::string &child_frame_id, const std::string &frame_id, Pose &pose) {
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set header and frame information
    // transform_stamped.header.stamp = pose.timestamp;
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

void map_rosbag_cam_poses(int argc, char **argv, bool dense) {
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
    double tolerance = 0.1; // Set tolerance

    // Initialize variables to store rosbag cameras images
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_cam0 = nullptr;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_cam1 = nullptr;
    rclcpp::Time timestamp_cam0;
    rclcpp::Time timestamp_cam1;

    // Get poses of the cameras
    Pose left_cam_pose_wrt_body = get_left_cam_pose_wrt_body();
    Pose right_cam_pose_wrt_body = get_right_cam_pose_wrt_body();
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
                RCLCPP_INFO(node->get_logger(), "Images are synchronized: %f",
                            timestamp_cam0.seconds());

                // Convert sensor_msgs::Image to cv::Mat using cv_bridge
                cv::Mat image_cam0 =
                    cv_bridge::toCvCopy(image_msg_cam0, sensor_msgs::image_encodings::BGR8)->image;
                cv::Mat image_cam1 =
                    cv_bridge::toCvCopy(image_msg_cam1, sensor_msgs::image_encodings::BGR8)->image;

                Pose closest_pose;
                bool is_close_pose =
                    findClosestPose(poses, timestamp_cam0.seconds(), tolerance, &closest_pose);

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
                    process_images_dense(image_cam0, image_cam1, points3D, block);
                else
                    process_images(image_cam0, image_cam1, R_estimated, T_estimated, points3D,
                                   block);

                if (!R_estimated.empty() && !T_estimated.empty()) {
                    T_estimated *= base_line_btw_cams;
                    Pose right_cam_pose_wrt_left_cam = createPose(R_estimated, T_estimated);
                    publish_camera_pose(tf_broadcaster_, nullptr, "right_cam_est", "left_cam",
                                        right_cam_pose_wrt_left_cam);
                }

                sensor_msgs::msg::PointCloud2 cloud_msg = points3DtoCloudMsg(points3D, "left_cam");

                publisher->publish(cloud_msg);
            } else {
                // If timestamps do not match, log the mismatch
                RCLCPP_INFO(node->get_logger(), "Timestamp mismatch: cam0 = %f, cam1 = %f",
                            timestamp_cam0.seconds(), timestamp_cam1.seconds());
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

Pose change_ref_system(Pose &pose_in, Pose &pose_ref) {
    tf2::Vector3 translation(pose_ref.x, pose_ref.y, pose_ref.z);
    tf2::Quaternion rotation(pose_ref.qx, pose_ref.qy, pose_ref.qz, pose_ref.qw);

    // Transform the position
    tf2::Vector3 position_in(pose_in.x, pose_in.y, pose_in.z);
    tf2::Vector3 position_rotated = tf2::quatRotate(rotation, position_in);
    tf2::Vector3 position_out = position_rotated + translation;

    // Transform the orientation
    tf2::Quaternion orientation_in(pose_in.qx, pose_in.qy, pose_in.qz, pose_in.qw);
    tf2::Quaternion orientation_out = rotation * orientation_in;

    // Set the transformed pose
    Pose pose_out;
    pose_out.x = position_out.x();
    pose_out.y = position_out.y();
    pose_out.z = position_out.z();
    pose_out.qx = orientation_out.x();
    pose_out.qy = orientation_out.y();
    pose_out.qz = orientation_out.z();
    pose_out.qw = orientation_out.w();

    return pose_out;
}

void correct_way(cv::Mat &T_estimated, const Pose &body_pose_wrt_map,
                 const Pose &new_body_pose_wrt_map) {
    // Calculate the displacement vector
    float delta_x = new_body_pose_wrt_map.x - body_pose_wrt_map.x;
    float delta_y = new_body_pose_wrt_map.y - body_pose_wrt_map.y;
    float delta_z = new_body_pose_wrt_map.z - body_pose_wrt_map.z;

    // Access T_estimated elements and adjust their signs based on displacement
    float &T_x = T_estimated.at<float>(0, 0); // x-component
    float &T_y = T_estimated.at<float>(1, 0); // y-component
    float &T_z = T_estimated.at<float>(2, 0); // z-component

    std::cout << "Before correcting T_estimated: [" << T_x << ", " << T_y << ", " << T_z << "]"
              << std::endl;

    if ((delta_x > 0 && T_x < 0) || (delta_x < 0 && T_x > 0)) {
        T_x = -T_x;
    }
    if ((delta_y > 0 && T_y < 0) || (delta_y < 0 && T_y > 0)) {
        T_y = -T_y;
    }
    if ((delta_z > 0 && T_z < 0) || (delta_z < 0 && T_z > 0)) {
        T_z = -T_z;
    }

    // Debugging output
    std::cout << "Corrected T_estimated: [" << T_x << ", " << T_y << ", " << T_z << "]"
              << std::endl;
}

void estimate_left_cam_trajectory(int argc, char **argv) {
    std::signal(SIGTSTP, handleSignal);

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("rosbag_reader");
    auto tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    // auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

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
    double tolerance = 0.1; // Set tolerance

    // Initialize variables to store rosbag cameras images
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_0 = nullptr;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_1 = nullptr;
    rclcpp::Time timestamp_image_0;
    rclcpp::Time timestamp_image_1;

    // Get poses of the cameras
    Pose cam_pose_wrt_body = get_left_cam_pose_wrt_body();
    Pose cam_pose_wrt_map = {-1, 0, 0, 0, 0, 0, 0, 1};
    Pose body_pose_wrt_map = {-1, 0, 0, 0, 0, 0, 0, 1};

    bool read_initial_pose = false;

    // Loop over the messages in the bag file
    while (rclcpp::ok() && reader.has_next()) {
        // Read the next serialized message
        auto bag_message = reader.read_next();

        rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

        if (bag_message->topic_name == "/cam0/image_raw" && !read_initial_pose) {
            image_msg_0 = std::make_shared<sensor_msgs::msg::Image>();
            image_serializer.deserialize_message(&serialized_msg, image_msg_0.get());
            timestamp_image_0 = rclcpp::Time(image_msg_0->header.stamp);

            bool is_close_pose_0 =
                findClosestPose(poses, timestamp_image_0.seconds(), tolerance, &body_pose_wrt_map);

            if (!is_close_pose_0)
                continue;

            cam_pose_wrt_map = change_ref_system(cam_pose_wrt_body, body_pose_wrt_map);

            read_initial_pose = true;

            publish_camera_pose(tf_broadcaster_, nullptr, "cam", "map", cam_pose_wrt_map);
            publish_camera_pose(tf_broadcaster_, nullptr, "body", "map", body_pose_wrt_map);

            continue;
        }

        if (bag_message->topic_name == "/cam0/image_raw" && image_msg_0) {
            image_msg_1 = std::make_shared<sensor_msgs::msg::Image>();
            image_serializer.deserialize_message(&serialized_msg, image_msg_1.get());
            timestamp_image_1 = rclcpp::Time(image_msg_1->header.stamp);
        }

        // If two images have been read
        if (image_msg_0 && image_msg_1) {
            // Convert sensor_msgs::Image to cv::Mat using cv_bridge
            cv::Mat image_0 =
                cv_bridge::toCvCopy(image_msg_0, sensor_msgs::image_encodings::BGR8)->image;
            cv::Mat image_1 =
                cv_bridge::toCvCopy(image_msg_1, sensor_msgs::image_encodings::BGR8)->image;

            Pose new_body_pose_wrt_map;
            bool is_close_pose = findClosestPose(poses, timestamp_image_1.seconds(), tolerance,
                                                 &new_body_pose_wrt_map);

            if (!is_close_pose) {
                std::cout << "Couldn't find close pose in ground truth." << std::endl;
                image_msg_1.reset();
                continue;
            }

            cv::Mat R_estimated, T_estimated;
            std::vector<cv::Point3d> points3D;

            process_images(image_0, image_1, R_estimated, T_estimated, points3D, block);

            if (!R_estimated.empty() && !T_estimated.empty()) {
                float distance_btw_poses =
                    get_distance_btw_poses(body_pose_wrt_map, new_body_pose_wrt_map);
                T_estimated *= distance_btw_poses;

                correct_way(T_estimated, body_pose_wrt_map, new_body_pose_wrt_map);

                Pose new_cam_pose_wrt_old_cam_pose = createPose(R_estimated, T_estimated);

                cam_pose_wrt_map =
                    change_ref_system(new_cam_pose_wrt_old_cam_pose, cam_pose_wrt_map);

                publish_camera_pose(tf_broadcaster_, nullptr, "cam", "map", cam_pose_wrt_map);
                publish_camera_pose(tf_broadcaster_, nullptr, "body", "map", new_body_pose_wrt_map);

                body_pose_wrt_map = new_body_pose_wrt_map;
                image_msg_0 = image_msg_1;
                image_msg_1.reset();
            }
        }
    }

    // Clean up and shutdown
    rclcpp::shutdown();
    return;
}
