FROM ros:humble-ros-core

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev \
    ros-humble-rclcpp \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-rosbag2-cpp \
    ros-humble-rosbag2-storage \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2 \
    ros-humble-ros2bag \
    ros-humble-rviz2 \
    libsqlite3-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src/app

COPY . .

WORKDIR /usr/src/app/build
RUN . /opt/ros/humble/setup.sh && cmake .. && make

RUN chmod +x /usr/src/app/entrypoint.sh

ENTRYPOINT ["/usr/src/app/entrypoint.sh"]
