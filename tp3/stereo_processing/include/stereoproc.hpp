#ifndef STEREOPROC_HPP
#define STEREOPROC_HPP

#include <opencv2/opencv.hpp>
#include <vector>

// Function to set up stereo camera matrices
void setupStereoCameraMatrices(cv::Mat& D_left, cv::Mat& K_left, cv::Mat& R_left, cv::Mat& P_left,
                               cv::Mat& D_right, cv::Mat& K_right, cv::Mat& R_right, cv::Mat& P_right,
                               cv::Mat& T, cv::Mat& R);

// Function to rectify stereo images
void rectify_images(const cv::Mat imgLeft, const cv::Mat imgRight, cv::Mat* rectifiedLeft, cv::Mat* rectifiedRight);

// Function to detect keypoints in images
void detect_keypoints(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint>* keypoint1, std::vector<cv::KeyPoint>* keypoint2);

// Function to compute descriptors for keypoints
void compute_descriptors(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint> keypoint1, std::vector<cv::KeyPoint> keypoint2, cv::Mat* descriptors1, cv::Mat* descriptors2);

// Function to match descriptors between two images
void match_descriptors(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::DMatch>* matches);

// Function to triangulate 3D points from matched 2D points
void triangulate(cv::Mat P_left, cv::Mat P_right, std::vector<cv::Point2f> pointsLeft, std::vector<cv::Point2f> pointsRight, std::vector<cv::Point3d>* points3D);

std::vector<cv::Point3d> triangulateKeyPoints(cv::Mat imgLeft, cv::Mat imgRight);

#endif // STEREOPROC_HPP
