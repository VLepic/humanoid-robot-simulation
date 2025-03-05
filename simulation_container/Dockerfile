# Base image
FROM kasmweb/core-cuda-focal:1.16.1

# Set environment variables for the container
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm

ARG ROBOTOLOGY_SUPERBUILD_VERSION=2024.11.2
ARG ERGOCUB_SOFTWARE_VERSION=0.7.6
ARG CMAKE_VERSION=3.31.5
ARG ROS2_DISTRO=foxy


USER root

# Clone the Robotology Superbuild & install Robotology Superbuild apt dependencies
RUN git clone https://github.com/robotology/robotology-superbuild.git /opt/robotology-superbuild && \
    cd /opt/robotology-superbuild && \
    git checkout v${ROBOTOLOGY_SUPERBUILD_VERSION} && \
    apt-get update && \
    apt-get install -y $(cat apt.txt) && \
    apt-get install -y $(cat apt-python.txt) && \
    apt-get install -y libboost-all-dev doxygen

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
    apt install -y ros-${ROS2_DISTRO}-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete ros-${ROS2_DISTRO}-test-msgs python3-dev && \
    rm -rf /var/lib/apt/lists/* \

# ROS2 & python environment variables
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV PATH=${PATH}:/opt/ros/${ROS2_DISTRO}/bin
ENV PYTHONPATH=${PYTHONPATH}:/opt/ros/${ROS2_DISTRO}/lib/python3.8/site-packages
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/ros/${ROS2_DISTRO}/lib
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/lib/x86_64-linux-gnu
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/ros/${ROS2_DISTRO}/lib/x86_64-linux-gnu
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
ENV PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:/opt/ros/${ROS2_DISTRO}/lib/pkgconfig
ENV AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}:/opt/ros/${ROS2_DISTRO}
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/lib/x86_64-linux-gnu
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/robotology-superbuild/build-base/install
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/${ROS2_DISTRO}
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/lib/x86_64-linux-gnu
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/opt/ros/${ROS2_DISTRO}/share/test_msgs/cmake

#Initialize ROS2 and Gazebo
RUN rosdep init && rosdep update && \
    . /usr/share/gazebo/setup.sh

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
ENV XDG_DATA_DIRS=${XDG_DATA_DIRS}:/usr/local/share/
ENV XDG_DATA_DIRS=${XDG_DATA_DIRS}:/usr/share/
ENV XDG_DATA_DIRS=${XDG_DATA_DIRS}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share

#YARP - Profile-specific configuration steps
#https://github.com/robotology/robotology-superbuild/blob/master/doc/environment-variables-configuration.md
#Dynamics
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/lib/robottestingframework
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/robotology/icub-tests/suites
ENV BLOCKTEST_RESOURCE_PATH=${BLOCKTEST_RESOURCE_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/bin
#Python
ENV PYTHONPATH=${PYTHONPATH}:$ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX/lib/python3/dist-packages
ENV PYTHONPATH=${PYTHONPATH}:$ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX/lib/python3.8/site-packages

# Gazebo Classic configuration
#https://github.com/robotology/robotology-superbuild/blob/master/doc/environment-variables-configuration.md
ENV GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/lib
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/gazebo/models
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/iCub/robots
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/ergoCub/robots


#Build Robotology Superbuild
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && \
    cd /opt/robotology-superbuild && \
    git checkout v${ROBOTOLOGY_SUPERBUILD_VERSION} && \
    cd .. && \
    mkdir /opt/robotology-superbuild/build-base && \
    cd /opt/robotology-superbuild/build-base && \
    cmake .. \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DROBOTOLOGY_ENABLE_DYNAMICS=ON \
	    -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON \
	    -DROBOTOLOGY_USES_GAZEBO=ON \
	    -DROBOTOLOGY_USES_GZ=OFF \
	    -DROBOTOLOGY_USES_ROS=OFF \
	    -DROBOTOLOGY_USES_ROS2=ON \
	    -DROBOTOLOGY_USES_IGNITION=OFF \
	    -DROBOTOLOGY_USES_PYTHON=ON && \
    cmake --build . --config Release && \
    cmake --build . --target install

ARG ERGOCUB_SOFTWARE_VERSION=0.7.7
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share

# Clone and install ergoCub software
RUN git clone https://github.com/icub-tech-iit/ergocub-software.git /opt/ergocub-software && \
    cd /opt/ergocub-software && \
    git checkout v${ERGOCUB_SOFTWARE_VERSION} && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} .. && \
    make -j$(nproc) && \
    make install

# Clone and install unicycle footstep planner
RUN git clone https://github.com/robotology/unicycle-footstep-planner /opt/unicycle-footstep-planner && \
    cd /opt/unicycle-footstep-planner && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} .. && \
    make -j$(nproc) && \
    make install

ENV UnicyclePlanner_DIR=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$UnicyclePlanner_DIR/lib

# Clone and install walking controllers
RUN git clone https://github.com/robotology/walking-controllers.git /opt/walking-controllers && \
    cd /opt/walking-controllers && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} -DCMAKE_PREFIX_PATH=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX} .. && \
    make -j$(nproc) && \
    make install

#Walking controllers path
ENV WalkingControllers_INSTALL_DIR=${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}
ENV PATH=${PATH}:${WalkingControllers_INSTALL_DIR}/bin
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${WalkingControllers_INSTALL_DIR}/share/ICUBcontrib

# Creation of desktop icons
# Yarpserver
RUN echo '#!/bin/bash\nyarpserver' > /usr/local/bin/start_yarpserver.sh && \
    chmod +x /usr/local/bin/start_yarpserver.sh && \
# Gazebo
    echo '#!/bin/bash\ngazebo --verbose' > /usr/local/bin/start_gazebo.sh && \
    chmod +x /usr/local/bin/start_gazebo.sh && \
# Yarp-ROS LiDAR Bridge
    echo '#!/bin/bash\nyarpdev --from /opt/yarp-ros2-ergocub-bridging/lidar_config.ini' > /usr/local/bin/start_yarp_ros_lidar_bridge.sh && \
    chmod +x /usr/local/bin/start_yarp_ros_lidar_bridge.sh && \
# YARP Server Desktop Entry
    echo "[Desktop Entry]\n\
Version=1.0\n\
Type=Application\n\
Name=YARP Server\n\
Exec=/usr/local/bin/start_yarpserver.sh\n\
Icon=utilities-terminal\n\
Terminal=true" > /usr/share/applications/yarpserver.desktop && \
    chmod +x /usr/share/applications/yarpserver.desktop && \
# Gazebo Desktop Entry
    echo "[Desktop Entry]\n\
Version=1.0\n\
Type=Application\n\
Name=Gazebo\n\
Exec=/usr/local/bin/start_gazebo.sh\n\
Icon=utilities-terminal\n\
Terminal=true" > /usr/share/applications/gazebo.desktop && \
    chmod +x /usr/share/applications/gazebo.desktop && \
# YARP-ROS LiDAR Bridge Desktop Entry
    echo "[Desktop Entry]\n\
Version=1.0\n\
Type=Application\n\
Name=YARP-ROS LiDAR Bridge\n\
Exec=/usr/local/bin/start_yarp_ros_lidar_bridge.sh\n\
Icon=utilities-terminal\n\
Terminal=true" > /usr/share/applications/yarp_ros_lidar_bridge.desktop && \
    chmod +x /usr/share/applications/yarp_ros_lidar_bridge.desktop && \
# Move to Desktop
    mkdir -p /home/kasm-user/Desktop && \
    cp /usr/share/applications/yarpserver.desktop /home/kasm-user/Desktop/ && \
    cp /usr/share/applications/gazebo.desktop /home/kasm-user/Desktop/ && \
    cp /usr/share/applications/yarp_ros_lidar_bridge.desktop /home/kasm-user/Desktop/ && \
    chmod +x /home/kasm-user/Desktop/*.desktop && \
    chown -R kasm-user:kasm-user /home/kasm-user/Desktop

# Set the default command to run the YARP server
CMD ["/usr/bin/supervisord && . /opt/ros/foxy/setup.sh"]