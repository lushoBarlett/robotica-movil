cmake_minimum_required(VERSION 3.5)
project(compvision)

# Find required packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)  # For cv_bridge
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)  # For reading/writing ROS bags
find_package(rosbag2_storage REQUIRED)  # For storage options in ROS bags
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


# Include directories
include_directories(include)

# Add executable
add_executable(compvision src/main.cpp src/bagreader.cpp src/stereoproc.cpp)

# Link dependencies
ament_target_dependencies(compvision
    rclcpp
    sensor_msgs
    cv_bridge  # For cv_bridge functionalities
    rosbag2_cpp  # For rosbag2_cpp functionalities
    rosbag2_storage  # For rosbag2_storage functionalities
    pcl_conversions
    PCL
)

# Link OpenCV libraries
target_link_libraries(compvision ${OpenCV_LIBS})

# Install targets
install(TARGETS
    compvision
    DESTINATION lib/${PROJECT_NAME})
