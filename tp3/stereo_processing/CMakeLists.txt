cmake_minimum_required(VERSION 3.5)
project(stereoproc)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(stereoproc src/stereoproc.cpp src/publisher.cpp)

ament_target_dependencies(stereoproc
    rclcpp
    sensor_msgs
    pcl_conversions
    PCL
)

target_link_libraries(stereoproc ${OpenCV_LIBS})

install(TARGETS
    stereoproc
    DESTINATION lib/${PROJECT_NAME})
