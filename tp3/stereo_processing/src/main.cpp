#include "mapping_node.hpp"
#include "stereoproc.hpp"
#include "trajectory_node.hpp"
#include <iostream>
#include <string>

void print_usage() {
    std::cout << "Usage: ./your_program <command> [args]" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  sparse" << std::endl;
    std::cout << "  dense" << std::endl;
    std::cout << "  traj" << std::endl;
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
    } else if (command == "sparse") {
        std::cout << "Performing sparse mapping..." << std::endl;
        rclcpp::init(argc, argv);

        auto node =
            std::make_shared<MappingNode>("../data/ros2.bag2", "../data/ground_truth.csv", false);

        node->process_bag();
    } else if (command == "dense") {
        std::cout << "Performing dense mapping..." << std::endl;
    } else if (command == "traj") {
        rclcpp::init(argc, argv);
        auto node =
            std::make_shared<TrajectoryNode>("../data/ros2.bag2", "../data/ground_truth.csv");

        node->process_bag();
        std::cout << "Performing trajectory estimation..." << std::endl;
    } else {
        std::cerr << "Error: Invalid command." << std::endl;
        print_usage();
        return 1;
    }

    return 0;
}
