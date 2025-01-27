# Base image
FROM kasmweb/core-cuda-focal:1.16.1

# Set environment variables for the container
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm

USER root

# Install prerequisites
RUN apt-get update && \
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
    libsqlite3-dev \
    libace-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

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
RUN wget https://github.com/Kitware/CMake/releases/download/v3.27.4/cmake-3.27.4-linux-x86_64.tar.gz && \
    tar -xzf cmake-3.27.4-linux-x86_64.tar.gz -C /opt/ && \
    ln -s /opt/cmake-3.27.4-linux-x86_64/bin/* /usr/local/bin/ && \
    rm cmake-3.27.4-linux-x86_64.tar.gz

# Configure Git
RUN git config --global user.name "Your Name" && \
    git config --global user.email "your.email@example.com"

# Install prerequisites and ROS 2 Galactic
RUN apt update && apt install -y \
    software-properties-common \
    curl \
    lsb-release \
    gnupg2 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt update && \
    apt install -y ros-galactic-ros-base python3-colcon-common-extensions python3-rosdep python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update


# Set up environment
ENV ROS_DISTRO=galactic
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3
ENV PATH="/opt/ros/galactic/bin:$PATH"
ENV PYTHONPATH="/opt/ros/galactic/lib/python3.8/site-packages:$PYTHONPATH"
ENV LD_LIBRARY_PATH=/opt/ros/galactic/lib:/opt/ros/galactic/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
ENV CMAKE_PREFIX_PATH="/opt/ros/galactic:$CMAKE_PREFIX_PATH"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/galactic/lib/x86_64-linux-gnu
ENV PKG_CONFIG_PATH=/opt/ros/galactic/lib/pkgconfig:$PKG_CONFIG_PATH
ENV AMENT_PREFIX_PATH=/opt/ros/galactic:$AMENT_PREFIX_PATH
ENV CMAKE_PREFIX_PATH="/lib/x86_64-linux-gnu:/opt/robotology-superbuild/build-base/install:/opt/ros/galactic:/usr/lib/x86_64-linux-gnu":$CMAKE_PREFIX_PATH


RUN apt-get update && apt-get install -y \
    python3-dev libboost-all-dev doxygen \
    build-essential cmake git

RUN apt-get install -y libpthread-stubs0-dev
ENV CMAKE_HAVE_LIBC_PTHREAD = TRUE
RUN apt-get update && apt-get install -y libc6-dev


ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
ENV LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LIBRARY_PATH

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libpthread-stubs0-dev \
    libboost-thread-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    ros-galactic-test-msgs


# Source ROS 2 setup script
RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc


# Clone the Robotology Superbuild
RUN git clone https://github.com/robotology/robotology-superbuild.git /opt/robotology-superbuild && \
    cd /opt/robotology-superbuild && \
    git checkout v2024.11.2 && \
    cd ..

# Build the base components (including YARP and core packages)
RUN mkdir /opt/robotology-superbuild/build-base && \
    cd /opt/robotology-superbuild/build-base && \
    cmake .. \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DENABLE_YARP:BOOL=ON \
        -DCMAKE_THREAD_LIBS_INIT="-lpthread" \
        -DCMAKE_HAVE_THREADS_LIBRARY="-lpthread" \
        -DTHREADS_PREFER_PTHREAD_FLAG=ON \
        -DPTHREADS_PREFER=ON \
        -DROBOTOLOGY_USES_GAZEBO=ON \
        -DROBOTOLOGY_USES_GZ=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DENABLE_yarpgui=ON \
        -DROBOTOLOGY_USES_ROS2=ON \
        -DROBOTOLOGY_USES_ROS=OFF \
        -DROBOTOLOGY_USES_OPENCV=ON \
        -DENABLE_GAZEBO:BOOL=OFF \
        -DENABLE_GAZEBO_CLASSIC:BOOL=OFF \
        -DENABLE_IDYNTREE:BOOL=ON \
        -DENABLE_WHOLEBODYDYNAMICS:BOOL=ON \
        -Dtest_msgs_DIR="/opt/ros/galactic/share/test_msgs/cmake" \
        -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON && \
    cmake --build . --config Release && \
    cmake --build . --target install

# Build Gazebo Classic and Gazebo YARP plugins
RUN mkdir /opt/robotology-superbuild/build-gazebo && \
    cd /opt/robotology-superbuild/build-gazebo && \
    cmake .. \
        -DENABLE_YARP:BOOL=OFF \
        -DENABLE_GAZEBO:BOOL=ON \
        -DBUILD_SHARED_LIBS=ON \
        -DENABLE_GAZEBO_CLASSIC:BOOL=ON \
        -DENABLE_gazebo-yarp-plugins:BOOL=ON \
        -DCMAKE_PREFIX_PATH="/usr/lib/x86_64-linux-gnu/cmake/gazebo" && \
    cmake --build . --config Release && \
    cmake --build . --target install


# Configure YARP server
RUN mkdir -p /root/.config/yarp && \
    echo "nameserver 127.0.0.1 10000" > /root/.config/yarp/config.yml

# Clone and install ergoCub software
RUN git clone https://github.com/icub-tech-iit/ergocub-software.git /opt/ergocub-software && \
    cd /opt/ergocub-software && \
    git checkout v0.7.6 && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/opt/ergocub-software/install -DCMAKE_PREFIX_PATH=/opt/robotology-superbuild/build-base/install .. && \
    make -j$(nproc) && \
    make install


# Set initial environment variables for Robotology Superbuild
ENV YARP_DATA_DIRS=/opt/robotology-superbuild/build-base/install/share/yarp:/opt/robotology-superbuild/build-base/install/share/ICUB:$YARP_DATA_DIRS
ENV GAZEBO_MODEL_PATH=/opt/robotology-superbuild/build-gazebo/install/share/gazebo/models:$GAZEBO_MODEL_PATH
ENV PATH=/opt/robotology-superbuild/build-base/install/bin:/opt/robotology-superbuild/build-gazebo/install/bin:$PATH
ENV PATH=$PATH:/opt/robotology-superbuild/build-base/src/YARP/src


# Add ergoCub paths to environment variables
ENV YARP_DATA_DIRS=$YARP_DATA_DIRS:/opt/ergocub-software/install/share/ergoCub:/opt/ergocub-software/install/share/yarp
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ergocub-software/install/share/ergoCub/robots:/opt/ergocub-software/install/share
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/gazebo-yarp-plugins/build:$GAZEBO_PLUGIN_PATH
ENV GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/robotology-superbuild/build-gazebo/install/lib
ENV YARP_DATA_DIRS=$YARP_DATA_DIRS:/opt/gazebo-yarp-plugins/share/yarp
ENV YARP_PLUGIN_PATH=/opt/robotology-superbuild/build-base/install/lib/yarp:$YARP_PLUGIN_PATH


ENV XDG_RUNTIME_DIR=/run/user/0



# Clone the repository
RUN git clone https://github.com/robotology/yarp-devices-ros2.git /opt/robotology-superbuild/build-yarp-devices-ros2 && \
    cd /opt/robotology-superbuild/build-yarp-devices-ros2 && \
    mkdir build && \
    cd build && \
    # Configure with necessary options
    cmake .. \
        -DYARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs:BOOL=ON \
        -DYARP_ROS2_USE_SYSTEM_yarp_control_msgs:BOOL=ON \
        -DCMAKE_INSTALL_PREFIX=/opt/robotology-superbuild/build-base/install && \
    # Build the project
    cmake --build . -- -j$(nproc) && \
    # Install the project
    cmake --build . --target install && \
    # Export YARP devices to the environment
    echo "export YARP_DATA_DIRS=\$YARP_DATA_DIRS:/opt/robotology-superbuild/build-base/install/share/yarp" >> /root/.bashrc \
    echo "export AMENT_PREFIX_PATH=\$AMENT_PREFIX_PATH:/opt/robotology-superbuild/build-base/install" >> /root/.bashrc


ENV YARP_PLUGIN_PATH=/opt/robotology-superbuild/build-base/install/lib/yarp:$YARP_PLUGIN_PATH


# Build the ROS 2 messages
RUN cd /opt/robotology-superbuild/build-yarp-devices-ros2/ros2_interfaces_ws && \
    colcon build --install-base /opt/robotology-superbuild/build-base/install && \
    /bin/bash -c "source /opt/robotology-superbuild/build-base/install/setup.bash" && \
    echo "source /opt/robotology-superbuild/build-base/install/setup.bash" >> /root/.bashrc








# Expose the KasmVNC and YARP ports
EXPOSE 6901 10000

# Creation of desktop icons
RUN echo '#!/bin/bash\nyarpserver' > /usr/local/bin/start_yarpserver.sh && \
    chmod +x /usr/local/bin/start_yarpserver.sh

RUN echo '#!/bin/bash\ngazebo --verbose' > /usr/local/bin/start_gazebo.sh && \
    chmod +x /usr/local/bin/start_gazebo.sh

RUN echo "[Desktop Entry]\n\
Version=1.0\n\
Type=Application\n\
Name=YARP Server\n\
Exec=/usr/local/bin/start_yarpserver.sh\n\
Icon=utilities-terminal\n\
Terminal=true" > /usr/share/applications/yarpserver.desktop && \
    chmod +x /usr/share/applications/yarpserver.desktop

RUN echo "[Desktop Entry]\n\
Version=1.0\n\
Type=Application\n\
Name=Gazebo\n\
Exec=/usr/local/bin/start_gazebo.sh\n\
Icon=utilities-terminal\n\
Terminal=true" > /usr/share/applications/gazebo.desktop && \
    chmod +x /usr/share/applications/gazebo.desktop

RUN mkdir -p /home/kasm-user/Desktop && \
    cp /usr/share/applications/yarpserver.desktop /home/kasm-user/Desktop/ && \
    cp /usr/share/applications/gazebo.desktop /home/kasm-user/Desktop/ && \
    chmod +x /home/kasm-user/Desktop/*.desktop && \
    chown -R kasm-user:kasm-user /home/kasm-user/Desktop

# Set the default command to run the YARP server
CMD ["/usr/bin/supervisord"]
