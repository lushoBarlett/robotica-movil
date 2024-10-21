#ifndef STEREOPROC_HPP
#define STEREOPROC_HPP

#include <opencv2/opencv.hpp>

// The functions compute the whole process and show the results
void rectifiyng_process(cv::Mat imgLeft, cv::Mat imgRight);

void keypoints_process(cv::Mat imgLeft, cv::Mat imgRight);

void matching_process(cv::Mat imgLeft, cv::Mat imgRight);

void homography_process(cv::Mat imgLeft, cv::Mat imgRight);

void disparity_map_process(cv::Mat imgLeft, cv::Mat imgRight);

void pose_estimation_process(cv::Mat imgLeft, cv::Mat imgRight);

// These functions calculate a vector of 3D points given 2 images with different methods
std::vector<cv::Point3d> dense_point_cloud(cv::Mat imgLeft, cv::Mat imgRight);

std::vector<cv::Point3d> triangulateKeyPoints(cv::Mat imgLeft, cv::Mat imgRight);

#endif // STEREOPROC_HPP
