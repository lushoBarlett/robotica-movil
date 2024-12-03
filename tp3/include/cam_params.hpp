#ifndef CAM_PARAMS_HPP
#define CAM_PARAMS_HPP

#include "poses.hpp"
#include <opencv2/core.hpp>

void setupStereoCameraMatrices(cv::Mat &D_left, cv::Mat &K_left, cv::Mat &R_left, cv::Mat &P_left,
                               cv::Mat &D_right, cv::Mat &K_right, cv::Mat &R_right,
                               cv::Mat &P_right, cv::Mat &T, cv::Mat &R);

float get_base_line_btw_cams();

Pose get_left_cam_pose_wrt_body();

Pose get_right_cam_pose_wrt_body();

#endif // CAM_PARAMS_HPP
