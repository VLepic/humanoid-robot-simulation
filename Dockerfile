# Base image
FROM kasmweb/core-ubuntu-focal:1.16.1

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

# Clone the Robotology Superbuild
RUN git clone https://github.com/robotology/robotology-superbuild.git /opt/robotology-superbuild

# Build the base components (including YARP and core packages)
RUN mkdir /opt/robotology-superbuild/build-base && \
    cd /opt/robotology-superbuild/build-base && \
    cmake .. \
        -DENABLE_YARP:BOOL=ON \
        -DROBOTOLOGY_ENABLE_CORE=ON \
        -DROBOTOLOGY_USES_GAZEBO=ON \
        -DROBOTOLOGY_USES_GZ=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DENABLE_yarpgui=ON \
        -DROBOTOLOGY_USES_ROS2=ON \
        -DROBOTOLOGY_USES_ROS=OFF \
        -DROBOTOLOGY_USES_OPENCV=ON \
        -DENABLE_GAZEBO:BOOL=OFF \
        -DENABLE_GAZEBO_CLASSIC:BOOL=OFF \
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
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/opt/ergocub-software/install -DCMAKE_PREFIX_PATH=/opt/robotology-superbuild/build-base/install .. && \
    make -j$(nproc) && \
    make install

# Set initial environment variables for Robotology Superbuild
ENV YARP_DATA_DIRS=/opt/robotology-superbuild/build-base/install/share/yarp:/opt/robotology-superbuild/build-base/install/share/ICUB
ENV GAZEBO_MODEL_PATH=/opt/robotology-superbuild/build-gazebo/install/share/gazebo/models
ENV PATH=/opt/robotology-superbuild/build-base/install/bin:/opt/robotology-superbuild/build-gazebo/install/bin:$PATH
ENV PATH=$PATH:/opt/robotology-superbuild/build-base/src/YARP/src


# Add ergoCub paths to environment variables
ENV YARP_DATA_DIRS=$YARP_DATA_DIRS:/opt/ergocub-software/install/share/ergoCub:/opt/ergocub-software/install/share/yarp
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ergocub-software/install/share/ergoCub/robots:/opt/ergocub-software/install/share
ENV GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/gazebo-yarp-plugins/build
ENV GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/robotology-superbuild/build-gazebo/install/lib
ENV YARP_DATA_DIRS=$YARP_DATA_DIRS:/opt/gazebo-yarp-plugins/share/yarp

# Expose the KasmVNC and YARP ports
EXPOSE 6901 10000

# Set the default command to run the YARP server
CMD ["/usr/bin/supervisord"]




































