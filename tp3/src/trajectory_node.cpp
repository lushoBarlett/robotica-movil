#include "trajectory_node.hpp"
#include "cam_params.hpp"
#include "node_utils.hpp"
#include "poses.hpp"
#include "stereoproc.hpp"
#include <cv_bridge/cv_bridge.h>
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

TrajectoryNode::TrajectoryNode(const std::string &bag_path, const std::string &ground_truth_csv,
                               const std::string &cam_filename, const std::string &body_filename)
    : Node("traj_estimator_node") {

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    initialize_rosbag_reader(bag_path);

    poses_ = readGroundTruthCSV(ground_truth_csv);

    signal(SIGTSTP, handleSignal);
    cam_pose_wrt_body_ = get_left_cam_pose_wrt_body();

    cam_file.open(cam_filename);
    body_file.open(body_filename);
    cam_file
        << "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y "
           "[], q_RS_z []"
        << std::endl;

    body_file
        << "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y "
           "[], q_RS_z []"
        << std::endl;

    if (!cam_file.is_open() || !body_file.is_open()) {
        std::cerr << "Error opening files for dumping the poses" << std::endl;
        return;
    }
}

void TrajectoryNode::process_bag() {
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_0 = nullptr, image_msg_1 = nullptr;
    rclcpp::Time timestamp_image_0, timestamp_image_1;

    while (rclcpp::ok() && reader_->has_next()) {
        if (!process_bag_message(image_serializer, timestamp_image_0, image_msg_0,
                                 timestamp_image_1, image_msg_1)) {
            continue;
        }

        cv::Mat image_0 =
            cv_bridge::toCvCopy(image_msg_0, sensor_msgs::image_encodings::BGR8)->image;
        cv::Mat image_1 =
            cv_bridge::toCvCopy(image_msg_1, sensor_msgs::image_encodings::BGR8)->image;

        process_images_and_update_pose(image_0, image_1, timestamp_image_1);
        image_msg_0 = image_msg_1;
        image_msg_1.reset();
    }
}

void TrajectoryNode::initialize_rosbag_reader(const std::string &bag_path) {
    reader_ = std::make_unique<rosbag2_cpp::Reader>(
        std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path;
    storage_options.storage_id = "sqlite3";
    reader_->open(storage_options);
}

bool TrajectoryNode::process_bag_message(
    rclcpp::Serialization<sensor_msgs::msg::Image> &image_serializer,
    rclcpp::Time &timestamp_image_0, std::shared_ptr<sensor_msgs::msg::Image> &image_msg_0,
    rclcpp::Time &timestamp_image_1, std::shared_ptr<sensor_msgs::msg::Image> &image_msg_1) {

    auto bag_message = reader_->read_next();
    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

    if (bag_message->topic_name == "/cam0/image_raw") {
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_serializer.deserialize_message(&serialized_msg, image_msg.get());

        if (!image_msg_0) {
            timestamp_image_0 = rclcpp::Time(image_msg->header.stamp);
            bool is_close_pose_0 = findClosestPose(poses_, timestamp_image_0.seconds(), tolerance_,
                                                   &body_pose_wrt_map_);
            if (!is_close_pose_0)
                return false;

            cam_pose_wrt_map_ = change_ref_system(cam_pose_wrt_body_, body_pose_wrt_map_);
            publish_pose(tf_broadcaster_, nullptr, "cam", "map", cam_pose_wrt_map_);
            publish_pose(tf_broadcaster_, nullptr, "body", "map", body_pose_wrt_map_);
            write_pose(cam_file, cam_pose_wrt_map_);
            write_pose(body_file, body_pose_wrt_map_);

            image_msg_0 = image_msg;
        } else {
            timestamp_image_1 = rclcpp::Time(image_msg->header.stamp);
            image_msg_1 = image_msg;
        }
    }

    return image_msg_0 && image_msg_1;
}

void TrajectoryNode::process_images_and_update_pose(cv::Mat &image_0, cv::Mat &image_1,
                                                    rclcpp::Time timestamp_image_1) {

    Pose new_body_pose_wrt_map;
    bool is_close_pose =
        findClosestPose(poses_, timestamp_image_1.seconds(), tolerance_, &new_body_pose_wrt_map);

    if (!is_close_pose) {
        std::cout << "Couldn't find close pose in ground truth." << std::endl;
        return;
    }

    cv::Mat R_estimated, T_estimated;
    monocular_process_images(image_0, image_1, R_estimated, T_estimated, block);

    if (!R_estimated.empty() && !T_estimated.empty()) {
        float distance_btw_poses =
            get_distance_btw_poses(body_pose_wrt_map_, new_body_pose_wrt_map);
        T_estimated *= -distance_btw_poses;

        Pose new_cam_pose_wrt_old_cam_pose = createPose(R_estimated, T_estimated);
        cam_pose_wrt_map_ = change_ref_system(new_cam_pose_wrt_old_cam_pose, cam_pose_wrt_map_);

        publish_pose(tf_broadcaster_, nullptr, "cam", "map", cam_pose_wrt_map_);
        publish_pose(tf_broadcaster_, nullptr, "body", "map", new_body_pose_wrt_map);
        write_pose(cam_file, cam_pose_wrt_map_);
        write_pose(body_file, new_body_pose_wrt_map);

        body_pose_wrt_map_ = new_body_pose_wrt_map;
    }
}
