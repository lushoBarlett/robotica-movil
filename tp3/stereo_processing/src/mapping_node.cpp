#include "mapping_node.hpp"
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
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

MappingNode::MappingNode(const std::string &bag_path, const std::string &poses_csv, bool dense,
                         bool estimate_displacement)
    : Node("mapping_node"), dense_(dense), estimate_displacement_(estimate_displacement) {
    // Initialize publishers, broadcasters and the rosbag reader
    point_cloud_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    initialize_rosbag_reader(bag_path);

    // Load poses and camera parameters
    poses_ = readGroundTruthCSV(poses_csv);
    base_line_btw_cams_ = get_base_line_btw_cams();

    signal(SIGTSTP, handleSignal);
    // Publish static transforms
    publish_camera_pose(nullptr, static_broadcaster_, "left_cam", "body",
                        get_left_cam_pose_wrt_body());
    publish_camera_pose(nullptr, static_broadcaster_, "right_cam", "body",
                        get_right_cam_pose_wrt_body());
}

void MappingNode::process_bag() {
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_cam0 = nullptr, image_msg_cam1 = nullptr;
    rclcpp::Time timestamp_cam0, timestamp_cam1;

    while (rclcpp::ok() && reader_->has_next()) {
        if (!process_bag_message(image_serializer, timestamp_cam0, image_msg_cam0, timestamp_cam1,
                                 image_msg_cam1)) {
            continue;
        }

        cv::Mat image_cam0 =
            cv_bridge::toCvCopy(image_msg_cam0, sensor_msgs::image_encodings::BGR8)->image;
        cv::Mat image_cam1 =
            cv_bridge::toCvCopy(image_msg_cam1, sensor_msgs::image_encodings::BGR8)->image;

        process_images(image_cam0, image_cam1, timestamp_cam0, timestamp_cam1);
        image_msg_cam0.reset();
        image_msg_cam1.reset();
    }
}

void MappingNode::initialize_rosbag_reader(const std::string &bag_path) {
    reader_ = std::make_unique<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";
    reader_->open(storage_options);
}

bool MappingNode::process_bag_message(
    rclcpp::Serialization<sensor_msgs::msg::Image> &image_serializer, rclcpp::Time &timestamp_cam0,
    std::shared_ptr<sensor_msgs::msg::Image> &image_msg_cam0, rclcpp::Time &timestamp_cam1,
    std::shared_ptr<sensor_msgs::msg::Image> &image_msg_cam1) {
    auto bag_message = reader_->read_next();
    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

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

    return image_msg_cam0 && image_msg_cam1;
}

void MappingNode::process_images(cv::Mat &image_cam0, cv::Mat &image_cam1,
                                 rclcpp::Time timestamp_cam0, rclcpp::Time timestamp_cam1) {
    if (timestamp_cam0 != timestamp_cam1)
        return;

    Pose closest_pose;
    bool is_close_pose =
        findClosestPose(poses_, timestamp_cam0.seconds(), tolerance_, &closest_pose);

    if (!is_close_pose) {
        RCLCPP_WARN(this->get_logger(), "Couldn't find close pose in ground truth.");
        return;
    }

    publish_camera_pose(tf_broadcaster_, nullptr, "body", "map", closest_pose);

    cv::Mat R_estimated, T_estimated;
    std::vector<cv::Point3d> points3D;

    if (dense_)
        stereo_process_images_dense(image_cam0, image_cam1, points3D, block);
    else
        stereo_process_images(image_cam0, image_cam1, R_estimated, T_estimated, points3D, block,
                              true, estimate_displacement_);

    if (!R_estimated.empty() && !T_estimated.empty()) {
        T_estimated *= base_line_btw_cams_;
        Pose right_cam_pose_wrt_left_cam = createPose(R_estimated, T_estimated);
        publish_camera_pose(tf_broadcaster_, nullptr, "right_cam_est", "left_cam",
                            right_cam_pose_wrt_left_cam);
    }

    auto cloud_msg = points3DtoCloudMsg(points3D, "left_cam");
    point_cloud_publisher_->publish(cloud_msg);
}
