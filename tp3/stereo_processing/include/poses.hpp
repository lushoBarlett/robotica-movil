#ifndef POSE_UTILS_HPP
#define POSE_UTILS_HPP

#include <opencv2/core.hpp>
#include <string>
#include <vector>

// Define Pose structure
struct Pose {
    double timestamp;
    float x, y, z;
    float qx, qy, qz, qw;
};

Pose createPose(const cv::Mat &R_estimated, const cv::Mat &T_estimated);
void printPose(const Pose &pose);
std::vector<Pose> readGroundTruthCSV(const std::string &csv_file);
bool findClosestPose(const std::vector<Pose> poses, double ros_time_seconds, double tolerance,
                     Pose *closest_pose);
float get_distance_btw_poses(Pose &body_pose_wrt_map, Pose &new_body_pose_wrt_map);

#endif // POSE_UTILS_HPP
