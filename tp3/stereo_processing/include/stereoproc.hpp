#ifndef STEREOPROC_HPP
#define STEREOPROC_HPP

#include <opencv2/core.hpp>

void process_images(cv::Mat imgLeft, cv::Mat imgRight, cv::Mat* R_estimated, cv::Mat* T_estimated, std::vector<cv::Point3d>* points3D, bool block);

void process_images_dense(cv::Mat imgLeft, cv::Mat imgRight, std::vector<cv::Point3d>* densePoints3D, bool block);

#endif // STEREOPROC_HPP
