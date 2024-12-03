#include "mapping_node.hpp"
#include "stereoproc.hpp"
#include "trajectory_node.hpp"
#include <iostream>
#include <string>

void print_usage() {
    std::cout << "Usage: ./your_program <command> [args]" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  1 (feature triangulation)" << std::endl;
    std::cout << "  2 (feature triangulation and pose estimation)" << std::endl;
    std::cout << "  3 (dense triangulation)" << std::endl;
    std::cout << "  4 (trajectory estimation with monocular vision)" << std::endl;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Error: No command provided." << std::endl;
        print_usage();
        return 1;
    }

    std::string command = argv[1];

    if (command == "--help" || command == "-h") {
        print_usage();
        return 0;
    } else if (command == "1") {
        std::cout << "Performing feature triangulation..." << std::endl;
        rclcpp::init(argc, argv);

        bool dense = false;
        bool pose_estimation = false;
        auto node = std::make_shared<MappingNode>("../data/ros2.bag2", "../data/ground_truth.csv",
                                                  dense, pose_estimation);

        node->process_bag();
    } else if (command == "2") {
        std::cout << "Performing feature triangulation and pose estimation..." << std::endl;
        rclcpp::init(argc, argv);

        bool dense = false;
        bool pose_estimation = true;
        auto node = std::make_shared<MappingNode>("../data/ros2.bag2", "../data/ground_truth.csv",
                                                  dense, pose_estimation);

        node->process_bag();
    } else if (command == "3") {
        std::cout << "Performing dense triangulation..." << std::endl;
        rclcpp::init(argc, argv);

        bool dense = true;
        bool pose_estimation = false;
        auto node = std::make_shared<MappingNode>("../data/ros2.bag2", "../data/ground_truth.csv",
                                                  dense, pose_estimation);

        node->process_bag();
    } else if (command == "4") {
        std::cout << "Performing trajectory estimation with monocular vision..." << std::endl;
        rclcpp::init(argc, argv);
        auto node = std::make_shared<TrajectoryNode>(
            "../data/ros2.bag2", "../data/ground_truth.csv",
            "../data/trajectory_estimation/cam.csv", "../data/trajectory_estimation/body.csv");

        node->process_bag();
    } else {
        std::cerr << "Error: Invalid command." << std::endl;
        print_usage();
        return 1;
    }

    return 0;
}
