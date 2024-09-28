FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

# Set up environment variables
ENV ROS_DISTRO humble
ENV ROS_WS /dev_ws
ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

# Create the workspace
RUN mkdir -p $ROS_WS/src
COPY cylinder_detector_pkg $ROS_WS/src/
WORKDIR $ROS_WS
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# Source the ROS 2 workspace in the container
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

# Copy the launch and the world model
COPY launch.py .
COPY world.model .

# Set the entrypoint for running ROS 2 commands
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS_WS/install/setup.bash && source /usr/share/gazebo/setup.bash && ros2 launch launch.py"]
