# Use the official OSRF ROS 2 Foxy Docker image as the base image
FROM osrf/ros:humble-desktop-full

# Set the working directory to /ros2_ws
WORKDIR /my_monkey

# Clone your ROS 2 package into the workspace
RUN mkdir -p src
COPY ./ /my_monkey/src/DMKE/

# Install additional dependencies if needed
# RUN apt-get update && apt-get install -y \
#     <package-name>

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Source the setup script in every new shell
RUN echo "source /my_monkey/install/setup.bash" >> /root/.bashrc
