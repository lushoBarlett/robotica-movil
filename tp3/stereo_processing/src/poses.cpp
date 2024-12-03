#include "poses.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <sstream>
#include <string>
#include <vector>

static inline float SIGN(float x) { return (x >= 0.0f) ? +1.0f : -1.0f; }

static inline float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

// quaternion = [x, y, z, w]'
static cv::Mat mRot2Quat(const cv::Mat &m) {
    float r11 = m.at<float>(0, 0);
    float r12 = m.at<float>(0, 1);
    float r13 = m.at<float>(0, 2);
    float r21 = m.at<float>(1, 0);
    float r22 = m.at<float>(1, 1);
    float r23 = m.at<float>(1, 2);
    float r31 = m.at<float>(2, 0);
    float r32 = m.at<float>(2, 1);
    float r33 = m.at<float>(2, 2);
    float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
    float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
    float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
    if (q0 < 0.0f) {
        q0 = 0.0f;
    }
    if (q1 < 0.0f) {
        q1 = 0.0f;
    }
    if (q2 < 0.0f) {
        q2 = 0.0f;
    }
    if (q3 < 0.0f) {
        q3 = 0.0f;
    }
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);
    if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
        q0 *= +1.0f;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    } else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0f;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    } else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0f;
        q3 *= SIGN(r32 + r23);
    } else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0f;
    } else {
        printf("coding error\n");
    }
    float r = NORM(q0, q1, q2, q3);
    q0 /= r;
    q1 /= r;
    q2 /= r;
    q3 /= r;

    cv::Mat res = (cv::Mat_<float>(4, 1) << q1, q2, q3, q0);
    return res;
}

Pose createPose(const cv::Mat &R_estimated, const cv::Mat &T_estimated) {
    Pose pose;

    // Set translation values
    pose.x = T_estimated.at<float>(0, 0);
    pose.y = T_estimated.at<float>(1, 0);
    pose.z = T_estimated.at<float>(2, 0);

    cv::Mat quat = mRot2Quat(R_estimated);

    pose.qx = quat.at<float>(0, 0);
    pose.qy = quat.at<float>(1, 0);
    pose.qz = quat.at<float>(2, 0);
    pose.qw = quat.at<float>(3, 0);

    return pose;
}

void printPose(const Pose &pose) {
    std::cout << "Pose:" << std::endl;
    std::cout << "  Timestamp: " << pose.timestamp << std::endl;
    std::cout << "  Position: (" << pose.x << ", " << pose.y << ", " << pose.z << ")" << std::endl;
    std::cout << "  Orientation (Quaternion): (" << pose.qx << ", " << pose.qy << ", " << pose.qz
              << ", " << pose.qw << ")" << std::endl;
}

// Function to read the CSV file and store the poses
std::vector<Pose> readGroundTruthCSV(const std::string &csv_file) {
    std::ifstream file(csv_file);
    std::string line;
    std::vector<Pose> poses;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Pose pose;
        char separator;

        // Assuming CSV format: timestamp, x, y, z, qw, qx, qy, qz
        ss >> pose.timestamp >> separator >> pose.x >> separator >> pose.y >> separator >> pose.z >>
            separator >> pose.qw >> separator >> pose.qx >> separator >> pose.qy >> separator >>
            pose.qz;

        pose.timestamp = static_cast<double>(pose.timestamp) / 1e9;

        poses.push_back(pose);
    }

    return poses;
}

// Function to find the closest pose for a given ROS timestamp in seconds
bool findClosestPose(const std::vector<Pose> poses, double ros_time_seconds, double tolerance,
                     Pose *closest_pose) {
    double min_diff = std::numeric_limits<double>::max();

    for (const auto &pose : poses) {
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

float get_distance_btw_poses(Pose &body_pose_wrt_map, Pose &new_body_pose_wrt_map) {
    float dx = new_body_pose_wrt_map.x - body_pose_wrt_map.x;
    float dy = new_body_pose_wrt_map.y - body_pose_wrt_map.y;
    float dz = new_body_pose_wrt_map.z - body_pose_wrt_map.z;

    return sqrt(dx * dx + dy * dy + dz * dz);
}

void write_pose(std::ofstream &file, const Pose &pose) {
    file << pose.timestamp << "," << pose.x << "," << pose.y << "," << pose.z << "," << pose.qw
         << "," << pose.qx << "," << pose.qy << "," << pose.qz << std::endl;
}
