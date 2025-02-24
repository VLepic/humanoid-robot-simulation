# Base image
FROM kasmweb/core-ubuntu-jammy:1.16.1

# Set environment variables for the container
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm \
    ROS_DISTRO=humble

USER root

# Install prerequisites
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
    build-essential \
    wget \
    software-properties-common \
    git \
    curl \
    libboost-all-dev \
    libeigen3-dev \
    libtinyxml2-dev \
    libopencv-dev \
    lsb-release \
    gnupg \
    gnupg2 \
    libsqlite3-dev \
    libace-dev  \
    libc6-dev \
    libpthread-stubs0-dev \
    libboost-thread-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libeigen3-dev \
    libxml2-dev \
    coinor-libipopt-dev \
    libassimp-dev \
    libirrlicht-dev \
    libglfw3-dev \
    ignition-fortress && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install a newer version of CMake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.31.5/cmake-3.31.5-linux-x86_64.tar.gz && \
    tar -xzf cmake-3.31.5-linux-x86_64.tar.gz -C /opt/ && \
    ln -s /opt/cmake-3.31.5-linux-x86_64/bin/* /usr/local/bin/ && \
    rm cmake-3.31.5-linux-x86_64.tar.gz

# Configure Git
RUN git config --global user.name "Your Name" && \
    git config --global user.email "your.email@example.com"


# Přidání ROS 2 repozitáře a jeho GPG klíče
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# Aktualizace a instalace ROS 2 Humble Desktop a vývojových nástrojů
RUN apt update && apt install -y \
    python3-dev \
    libboost-all-dev \
    doxygen \
    build-essential \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

# Nastavení prostředí pro ROS 2
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV PATH="/opt/ros/humble/bin:$PATH"
ENV PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH="/opt/ros/humble:$CMAKE_PREFIX_PATH"
ENV PKG_CONFIG_PATH=/opt/ros/humble/lib/pkgconfig:$PKG_CONFIG_PATH
ENV AMENT_PREFIX_PATH=/opt/ros/humble:$AMENT_PREFIX_PATH
ENV CMAKE_PREFIX_PATH="/lib/x86_64-linux-gnu:/opt/robotology-superbuild/build-base/install:/opt/ros/humble:/usr/lib/x86_64-linux-gnu:/opt/ros/humble/share/test_msgs/cmake":$CMAKE_PREFIX_PATH

RUN apt-get update && apt-get install -y \
    swig \
    libeigen3-dev \
    libopencv-dev \
    libboost-all-dev \
    python3-dev \
    python3-pip

RUN apt install -y qtbase5-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libboost-all-dev ros-humble-test-msgs ros-humble-rclpy libcli11-dev libmatio-dev

#YARP - Generic configuration
#https://github.com/robotology/robotology-superbuild/blob/master/doc/environment-variables-configuration.md
ENV ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX=/opt/robotology-superbuild/build-base/install
ENV PATH=${PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/bin
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/yarp
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/iCub
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/ergoCub
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/ICUBcontrib
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/lib
ENV AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}
ENV XDG_DATA_DIRS=${XDG_DATA_DIRS}:/usr/local/share/:/usr/share/:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share

#YARP - Profile-specific configuration steps
#https://github.com/robotology/robotology-superbuild/blob/master/doc/environment-variables-configuration.md
#Dynamics
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/lib/robottestingframework
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/robotology/icub-tests/suites
ENV BLOCKTEST_RESOURCE_PATH=${BLOCKTEST_RESOURCE_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/bin


# Gazebo Harmonic Paths
# Modern Gazebo (gz-sim) related env variables (see https://gazebosim.org/api/sim/8/resources.html)
#https://github.com/robotology/robotology-superbuild/blob/master/doc/environment-variables-configuration.md
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/lib
ENV GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/gazebo/models
ENV GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/iCub/robots
ENV GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share
ENV GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/ergoCub/robots
ENV GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/gazebo/worlds

# Initialize rosdep
RUN rosdep init && rosdep update && \
    . /opt/ros/humble/setup.sh && \
    git clone https://github.com/robotology/robotology-superbuild.git /opt/robotology-superbuild && \
    cd /opt/robotology-superbuild && \
    git checkout v2024.11.2 && \
    cd .. && \
    mkdir /opt/robotology-superbuild/build-base && \
    cd /opt/robotology-superbuild/build-base && \
    cmake .. \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DROBOTOLOGY_ENABLE_DYNAMICS=OFF \
	    -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON \
	    -DROBOTOLOGY_USES_GAZEBO=OFF \
	    -DROBOTOLOGY_USES_GZ=OFF \
	    -DROBOTOLOGY_USES_ROS=OFF \
	    -DROBOTOLOGY_USES_ROS2=ON \
	    -DROBOTOLOGY_USES_IGNITION=ON \
	    -DROBOTOLOGY_USES_PYTHON=ON \
        -Dtest_msgs_DIR="/opt/ros/humble/share/test_msgs/cmake" && \
    cmake --build . --config Release && \
    cmake --build . --target install

# Clone and install ergoCub software
RUN git clone https://github.com/icub-tech-iit/ergocub-software.git /opt/ergocub-software && \
    cd /opt/ergocub-software && \
    git checkout v0.7.6 && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} .. && \
    make -j$(nproc) && \
    make install




# Set the default command to run the YARP server
CMD ["/usr/bin/supervisord && . /opt/ros/humble/setup.sh"]