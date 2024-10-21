#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "bagreader.hpp"
#include "stereoproc.hpp"

void print_usage() {
    std::cout << "Usage: ./your_program <command> [args]" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  rectifyImages" << std::endl;
    std::cout << "  keypoints" << std::endl;
    std::cout << "  matches" << std::endl;
    std::cout << "  homography" << std::endl;
    std::cout << "  mapping <mode> (sparse or dense)" << std::endl;
    std::cout << "  disparityMap" << std::endl;
    std::cout << "  estimatePose" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Error: No command provided." << std::endl;
        print_usage();
        return 1;
    }

    std::string command = argv[1];

    if (command == "--help" || command == "-h") {
        print_usage();
        return 0;
    }

    cv::Mat imgLeft = cv::imread("../data/left-0000.png", cv::COLOR_BGR2GRAY);
    cv::Mat imgRight = cv::imread("../data/right-0000.png", cv::COLOR_BGR2GRAY);

    if (command == "rectifyImages") {
        std::cout << "Running rectifyImages..." << std::endl;
        rectifiyng_process(imgLeft, imgRight);
    }
    else if (command == "keypoints") {
        std::cout << "Detecting keypoints..." << std::endl;
        keypoints_process(imgLeft, imgRight);
    }
    else if (command == "matches") {
        std::cout << "Matching keypoints..." << std::endl;
        matching_process(imgLeft, imgRight);
    }
    else if (command == "homography") {
        std::cout << "Computing homography..." << std::endl;
        homography_process(imgLeft, imgRight);
    }
    else if (command == "mapping") {
        if (argc < 3) {
            std::cerr << "Error: Missing mode for mapping (sparse or dense)." << std::endl;
            return 1;
        }

        std::string mode = argv[2];
        if (mode == "sparse") {
            std::cout << "Performing sparse mapping..." << std::endl;
            map_rosbag(argc, argv, triangulateKeyPoints);
        }
        else if (mode == "dense") {
            std::cout << "Performing dense mapping..." << std::endl;
            map_rosbag(argc, argv, dense_point_cloud);
        }
        else {
            std::cerr << "Error: Invalid mode for mapping. Choose 'sparse' or 'dense'." << std::endl;
            return 1;
        }
    }
    else if (command == "disparityMap") {
        std::cout << "Generating disparity map..." << std::endl;
        disparity_map_process(imgLeft, imgRight);
    }
    else if (command == "estimatePose") {
        std::cout << "Estimating pose..." << std::endl;
        pose_estimation_process(imgLeft, imgRight);
    }
    else {
        std::cerr << "Error: Invalid command." << std::endl;
        print_usage();
        return 1;
    }

    return 0;
}
