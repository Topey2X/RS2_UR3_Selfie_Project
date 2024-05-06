# Use the official ROS Noetic base image
FROM ros:noetic

# Install packages
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
RUN rosdep && rosdep update

# Set up the workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Optionally copy your ROS packages into the container
COPY ./src /catkin_ws/src

# Install dependencies for any packages in src
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Set the environment variables
ENV ROS_PACKAGE_PATH=/catkin_ws/src:$ROS_PACKAGE_PATH

# Set the entry point
# CMD ["roslaunch", "my_package my_launch_file.launch"]
