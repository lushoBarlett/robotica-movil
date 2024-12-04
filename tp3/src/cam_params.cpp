#include "poses.hpp"
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <opencv2/opencv.hpp>

cv::Mat left_cam_param_D;
cv::Mat left_cam_param_K;
cv::Mat left_cam_param_R;
cv::Mat left_cam_param_P;
cv::Mat right_cam_param_D;
cv::Mat right_cam_param_K;
cv::Mat right_cam_param_R;
cv::Mat right_cam_param_P;
cv::Mat cams_param_T;
cv::Mat cams_param_R;

float get_base_line_btw_cams() {
    return cv::norm(cams_param_t);
}

Pose get_left_cam_pose_wrt_body() {
    Pose pose = {0,          -0.0216401454975, -0.064676986768, 0.00981073058949,
                 -0.0077072, 0.0104993,        0.7017528,       0.7123015};

    return pose;
}

Pose get_right_cam_pose_wrt_body() {
    Pose pose = {0,          -0.0198435579556, 0.0453689425024, 0.00786212447038,
                 -0.0025502, 0.0153239,        0.7024867,       0.7115273};

    return pose;
}

static
std::vector<double> parseLine(const std::string& line) {
    std::vector<double> values;
    std::stringstream ss(line.substr(line.find('=') + 1));
    std::string number;

    while (std::getline(ss, number, ',')) {
        number.erase(std::remove(number.begin(), number.end(), '['), number.end());
        number.erase(std::remove(number.begin(), number.end(), ']'), number.end());
        number.erase(std::remove(number.begin(), number.end(), ' '), number.end());
        if (!number.empty()) {
            values.push_back(std::stod(number));
        }
    }

    return values;
}

void load_camera_parameters(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;
    bool is_left = false, is_right = false;

    while (std::getline(file, line)) {
        if (line.find("Left:") != std::string::npos) {
            is_left = true;
            is_right = false;
            continue;
        }
        if (line.find("Right:") != std::string::npos) {
            is_left = false;
            is_right = true;
            continue;
        }

        auto values = parseLine(line);
        if (line.find("self.T =") != std::string::npos) {
            cams_param_T = cv::Mat(3, 1, CV_64F, values.data()).clone();
        } else if (line.find("self.R =") != std::string::npos) {
            cams_param_R = cv::Mat(3, 3, CV_64F, values.data()).clone();
        } else if (line.find("D =") != std::string::npos) {
            if (is_left) left_cam_param_D = cv::Mat(1, 5, CV_64F, values.data()).clone();
            if (is_right) right_cam_param_D = cv::Mat(1, 5, CV_64F, values.data()).clone();
        } else if (line.find("K =") != std::string::npos) {
            if (is_left) left_cam_param_K = cv::Mat(3, 3, CV_64F, values.data()).clone();
            if (is_right) right_cam_param_K = cv::Mat(3, 3, CV_64F, values.data()).clone();
        } else if (line.find("R =") != std::string::npos) {
            if (is_left) left_cam_param_R = cv::Mat(3, 3, CV_64F, values.data()).clone();
            if (is_right) right_cam_param_R = cv::Mat(3, 3, CV_64F, values.data()).clone();
        } else if (line.find("P =") != std::string::npos) {
            if (is_left) left_cam_param_P = cv::Mat(3, 4, CV_64F, values.data()).clone();
            if (is_right) right_cam_param_P = cv::Mat(3, 4, CV_64F, values.data()).clone();
        }
    }

    file.close();
}
