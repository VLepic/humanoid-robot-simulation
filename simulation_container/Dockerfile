# Base image
FROM kasmweb/core-cuda-focal:1.16.1

# Set environment variables for the container
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm

ARG ROBOTOLOGY_SUPERBUILD_VERSION=2024.11.2
ARG ERGOCUB_SOFTWARE_VERSION=v3.4.0
ARG CMAKE_VERSION=3.31.5
ARG ROS2_DISTRO=galactic


USER root

# Clone the Robotology Superbuild & install Robotology Superbuild apt dependencies
RUN git clone https://github.com/robotology/robotology-superbuild.git /opt/robotology-superbuild && \
    cd /opt/robotology-superbuild && \
    git checkout v${ROBOTOLOGY_SUPERBUILD_VERSION} && \
    apt-get update && \
    apt-get install -y $(cat apt.txt) && \
    apt-get install -y $(cat apt-python.txt)

# Install Gazebo Classic and its development libraries
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install a newer version of CMake
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz && \
    tar -xzf cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz -C /opt/ && \
    ln -s /opt/cmake-${CMAKE_VERSION}-linux-x86_64/bin/* /usr/local/bin/ && \
    rm cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz

# Configure Git
RUN git config --global user.name "Your Name" && \
    git config --global user.email "your.email@example.com"

# Install prerequisites and ROS 2
RUN apt update && apt install -y \
    software-properties-common \
    curl \
    lsb-release \
    gnupg2 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt update && \
    apt install -y ros-${ROS2_DISTRO}-ros-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# Set the default command to run the YARP server
CMD ["/usr/bin/supervisord"]