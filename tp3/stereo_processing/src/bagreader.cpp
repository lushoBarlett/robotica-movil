#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/viz.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include "stereoproc.hpp"

// Struct to hold pose information
struct Pose {
    double timestamp;
    // Add fields for position and orientation (e.g., x, y, z, quaternion, etc.)
    double x, y, z, qx, qy, qz, qw;
};

void to_homogenous_coords(const std::vector<cv::Point3d>& points3D, std::vector<cv::Mat>& points3DHomogenous) {
    for (const auto& point : points3D) {
        cv::Mat pointHomogenous = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
        points3DHomogenous.push_back(pointHomogenous);
    }
}

void from_homogenous_coords(const std::vector<cv::Mat>& points3DHomogenous, std::vector<cv::Point3d>& points3D) {
    for (const auto& point : points3DHomogenous) {
        cv::Point3d point3D;
        float w = point.at<float>(3, 0);
        point3D.x = point.at<float>(0, 0) / w;
        point3D.y = point.at<float>(1, 0) / w;
        point3D.z = point.at<float>(2, 0) / w;
        points3D.push_back(point3D);
    }
}

cv::Mat pose2rot(Pose pose) {
    double w = pose.qw;
    double x = pose.qx;
    double y = pose.qy;
    double z = pose.qz;

    cv::Mat R = (cv::Mat_<double>(3, 3) << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
                                           2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
                                           2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y);

    return R;
}

cv::Mat poseToTransformationMatrix(const Pose pose) {
    // Create a 4x4 identity matrix
    cv::Mat transformation_matrix = cv::Mat::eye(4, 4, CV_32F);

    // Convert the quaternion to a rotation matrix
    // OpenCV represents quaternions as (qw, qx, qy, qz)
    cv::Mat rotation_matrix = pose2rot(pose);

    // Fill the top-left 3x3 block with the rotation matrix
    rotation_matrix.copyTo(transformation_matrix(cv::Rect(0, 0, 3, 3)));

    // Set the translation vector in the last column
    transformation_matrix.at<float>(0, 3) = pose.x;
    transformation_matrix.at<float>(1, 3) = pose.y;
    transformation_matrix.at<float>(2, 3) = pose.z;

    printMat(transformation_matrix);
    return transformation_matrix;
}

// Function to read the CSV file and store the poses
std::vector<Pose> readGroundTruthCSV(const std::string& csv_file) {
    std::ifstream file(csv_file);
    std::string line;
    std::vector<Pose> poses;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Pose pose;
        char separator;

        // Assuming CSV format: timestamp, x, y, z, qx, qy, qz, qw
        ss >> pose.timestamp >> separator >> pose.x >> separator >> pose.y >> separator
           >> pose.z >> separator >> pose.qw >> separator >> pose.qx >> separator >> pose.qy >> separator
           >> pose.qz;
        
        pose.timestamp = static_cast<double>(pose.timestamp) / 1e9;

        std::cout << std::fixed << std::setprecision(9) << "Timestamp (seconds): " << pose.timestamp << "\n";

        poses.push_back(pose);
    }

    return poses;
}

// Function to find the closest pose for a given ROS timestamp in seconds
bool findClosestPose(const std::vector<Pose> poses, double ros_time_seconds, double tolerance, Pose* closest_pose) {
    double min_diff = std::numeric_limits<double>::max();  // Initialize with a large value

    for (const auto& pose : poses) {
        double time_diff = std::fabs(pose.timestamp - ros_time_seconds);
        if (time_diff < min_diff && time_diff <= tolerance) {
            min_diff = time_diff;
            *closest_pose = pose;
        } else if (ros_time_seconds < pose.timestamp) {
            break;
        }
    }

    return (min_diff <= tolerance);
}

sensor_msgs::msg::PointCloud2 points3DtoCloudMsg(const std::vector<cv::Point3d>& points3D) {
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

    return cloud_msg;
}

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = std::make_shared<rclcpp::Node>("rosbag_reader");
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);

    // Create a reader object for the ROS bag
    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());

    // Set storage options and path to your ROS bag file
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "../data/ros2.bag2";  // Specify your ROS 2 bag file here
    storage_options.storage_id = "sqlite3";  // The default storage format is SQLite3

    // Open the ROS bag file
    reader.open(storage_options);

    std::string csv_file = "../data/ground_truth.csv";
    std::vector<Pose> poses = readGroundTruthCSV(csv_file);
    double tolerance = 0.1;  // Set tolerance

    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_cam0 = nullptr;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_cam1 = nullptr;
    rclcpp::Time timestamp_cam0;
    rclcpp::Time timestamp_cam1;

    // Loop over the messages in the bag file
    while (reader.has_next()) {
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
            // Deserialize the message to sensor_msgs::msg::Image
            image_msg_cam1 = std::make_shared<sensor_msgs::msg::Image>();
            image_serializer.deserialize_message(&serialized_msg, image_msg_cam1.get());
            timestamp_cam1 = rclcpp::Time(image_msg_cam1->header.stamp);
        }

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

                cv::Mat transformation = poseToTransformationMatrix(closest_pose);

                std::vector<cv::Point3d> points3D = triangulateKeyPoints(image_cam0, image_cam1);

                std::vector<cv::Mat> points3DHomogenous;
                to_homogenous_coords(points3D, points3DHomogenous);

                for (auto& pointH : points3DHomogenous) {
                    transformation.convertTo(transformation, CV_32F);
                    pointH.convertTo(pointH, CV_32F);
                    pointH = transformation * pointH;
                }

                std::vector<cv::Point3d> points3DMoved;
                from_homogenous_coords(points3DHomogenous, points3DMoved);

                sensor_msgs::msg::PointCloud2 cloud_msg = points3DtoCloudMsg(points3DMoved);

                publisher->publish(cloud_msg);

                // Display the images (optional)
                //cv::imshow("Image from /cam0/image_raw", image_cam0);
                //cv::imshow("Image from /cam1/image_raw", image_cam1);
                //cv::waitKey(0);  // Wait for key press before proceeding
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
    return 0;
}
