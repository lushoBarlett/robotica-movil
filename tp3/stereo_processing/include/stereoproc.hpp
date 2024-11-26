#ifndef STEREOPROC_HPP
#define STEREOPROC_HPP

#include <opencv2/core.hpp>

void stereo_process_images(cv::Mat &imgLeft, cv::Mat &imgRight, cv::Mat &R_estimated,
                           cv::Mat &T_estimated, std::vector<cv::Point3d> &points3D, bool block,
                           bool triangulate, bool estimate_displacement);

void stereo_process_images_dense(cv::Mat &imgLeft, cv::Mat &imgRight,
                                 std::vector<cv::Point3d> &densePoints3D, bool block);

#endif // STEREOPROC_HPP
