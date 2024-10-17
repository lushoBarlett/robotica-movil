#ifndef POINTCLOUD_PUBLISHER_HPP
#define POINTCLOUD_PUBLISHER_HPP

#include <vector>
#include <opencv2/core.hpp>
/**
 * @brief Publish a point cloud once using ROS2.
 * 
 * @param points3D A vector of 3D points (cv::Point3d) to be published.
 */
void publishContinuously(const std::vector<cv::Point3d>& points3D);

#endif  // POINTCLOUD_PUBLISHER_HPP
