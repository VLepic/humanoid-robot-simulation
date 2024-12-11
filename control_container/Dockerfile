FROM osrf/ros:galactic-desktop

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm \
    ROS_DISTRO=galactic

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-yarp-ros2-bridge \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    curl && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip and install additional Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    yarp-python \
    matplotlib \
    pandas

# Set up workspace
WORKDIR /root/ros2_ws

# Create and build a sample ROS 2 package
RUN mkdir -p src && cd src && \
    git clone https://github.com/your/control-package.git && \
    cd .. && \
    colcon build

# Add environment setup to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

EXPOSE 11311

# Default command
CMD ["bash"]
