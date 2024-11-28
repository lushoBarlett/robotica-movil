#include "node_utils.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

bool block = false;

void handleSignal(int signal) {
    if (signal == SIGTSTP) {
        block = !block;
    }
}

void publish_camera_pose(
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
    const std::shared_ptr<tf2_ros::StaticTransformBroadcaster> &static_broadcaster,
    const std::string &child_frame_id, const std::string &frame_id, const Pose &pose) {
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

sensor_msgs::msg::PointCloud2 points3DtoCloudMsg(const std::vector<cv::Point3d> &points3D,
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

void adjust_translation_sign(cv::Mat &T_estimated, const Pose &body_pose_wrt_map,
                             const Pose &new_body_pose_wrt_map) {
    // Calculate the displacement vector
    float delta_x = new_body_pose_wrt_map.x - body_pose_wrt_map.x;
    float delta_y = new_body_pose_wrt_map.y - body_pose_wrt_map.y;
    float delta_z = new_body_pose_wrt_map.z - body_pose_wrt_map.z;

    // Access T_estimated elements and adjust their signs based on displacement
    float &T_x = T_estimated.at<float>(0, 0); // x-component
    float &T_y = T_estimated.at<float>(1, 0); // y-component
    float &T_z = T_estimated.at<float>(2, 0); // z-component

    if ((delta_x > 0 && T_x < 0) || (delta_x < 0 && T_x > 0)) {
        T_x = -T_x;
    }
    if ((delta_y > 0 && T_y < 0) || (delta_y < 0 && T_y > 0)) {
        T_y = -T_y;
    }
    if ((delta_z > 0 && T_z < 0) || (delta_z < 0 && T_z > 0)) {
        T_z = -T_z;
    }
}
