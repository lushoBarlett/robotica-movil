#ifndef CAM_PARAMS_HPP
#define CAM_PARAMS_HPP

#include "poses.hpp"
#include <opencv2/core.hpp>

extern cv::Mat left_cam_param_D, left_cam_param_K, left_cam_param_R, left_cam_param_P;
extern cv::Mat right_cam_param_D, right_cam_param_K, right_cam_param_R, right_cam_param_P;
extern cv::Mat cams_param_T, cams_param_R;

void load_camera_parameters();

float get_base_line_btw_cams();

Pose get_left_cam_pose_wrt_body();

Pose get_right_cam_pose_wrt_body();

#endif // CAM_PARAMS_HPP
