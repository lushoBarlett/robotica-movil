#ifndef NODE_UTILS_HPP
#define NODE_UTILS_HPP

#include "poses.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <signal.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

extern bool block; // Flag to control pause state

void handleSignal(int signal);

void publish_camera_pose(
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster,
    const std::shared_ptr<tf2_ros::StaticTransformBroadcaster> &static_broadcaster,
    const std::string &child_frame_id, const std::string &frame_id, const Pose &pose);

sensor_msgs::msg::PointCloud2 points3DtoCloudMsg(const std::vector<cv::Point3d> &points3D,
                                                 const std::string &frame_id);

Pose change_ref_system(Pose &pose_in, Pose &pose_ref);

void fix_way(cv::Mat &T_estimated, const Pose &body_pose_wrt_map,
             const Pose &new_body_pose_wrt_map);

#endif // NODE_UTILS_HPP
