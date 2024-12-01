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

# Build the base components
RUN mkdir /opt/robotology-superbuild/build-base && \
    cd /opt/robotology-superbuild/build-base && \
    cmake .. \
        -DENABLE_YARP:BOOL=ON \
        -DENABLE_GAZEBO:BOOL=OFF \
        -DENABLE_GAZEBO_CLASSIC:BOOL=OFF \
        -DENABLE_gazebo-yarp-plugins:BOOL=OFF && \
    cmake --build . --config Release && \
    cmake --build . --target install


# Build Gazebo Classic and Gazebo YARP plugins
RUN mkdir /opt/robotology-superbuild/build-gazebo && \
    cd /opt/robotology-superbuild/build-gazebo && \
    cmake .. \
    -DENABLE_YARP:BOOL=OFF \
    -DENABLE_GAZEBO:BOOL=ON \
    -DENABLE_GAZEBO_CLASSIC:BOOL=ON \
    -DENABLE_gazebo-yarp-plugins:BOOL=ON \
    -DCMAKE_PREFIX_PATH="/usr/lib/x86_64-linux-gnu/cmake/gazebo11" && \
    cmake --build . --config Release && \
    cmake --build . --target install

# Set environment variables for Robotology Superbuild
ENV YARP_DATA_DIRS=/opt/robotology-superbuild/build-base/install/share/yarp:/opt/robotology-superbuild/build-base/install/share/ICUB \
    GAZEBO_MODEL_PATH=/opt/robotology-superbuild/build-gazebo/install/share/gazebo/models \
    PATH=/opt/robotology-superbuild/build-base/install/bin:/opt/robotology-superbuild/build-gazebo/install/bin:$PATH

# Expose the KasmVNC default port
EXPOSE 6901

# Set the default command or entrypoint
CMD ["/usr/bin/supervisord"]



































