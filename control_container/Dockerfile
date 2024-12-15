FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm \
    ROS_DISTRO=humble \
    WORKSPACE=/root/ros2_ws

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-rqt-console \
    ros-${ROS_DISTRO}-ros2doctor \
    ros-${ROS_DISTRO}-ros2bag \
    python3-babeltrace \
    cmake \
    git \
    curl \
    wget \
    libboost-all-dev \
    libeigen3-dev \
    libtinyxml2-dev \
    libace-dev \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Upgrade pip and install additional Python packages
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    matplotlib \
    pandas \
    notebook

# Configure Git globally
RUN git config --global user.name "Your Name" && \
    git config --global user.email "your.email@example.com"

# Reinstall YCM
RUN git clone https://github.com/robotology/ycm.git /opt/ycm && \
    mkdir -p /opt/ycm/build && cd /opt/ycm/build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr && \
    make -j$(nproc) && \
    make install

# Verify YCM installation
RUN ls /usr/share/YCM/modules && \
    ls /usr/share/YCM/modules/YCMEPHelper

# Clone and build YARP with Python bindings
RUN git clone https://github.com/robotology/yarp.git /opt/yarp && \
    mkdir -p /opt/yarp/build && cd /opt/yarp/build && \
    cmake .. -DCREATE_PYTHON=ON -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_PREFIX_PATH=/usr && \
    make -j$(nproc) && \
    make install

# Clone and build robotology-superbuild
RUN git clone https://github.com/robotology/robotology-superbuild.git /opt/robotology-superbuild && \
    mkdir -p /opt/robotology-superbuild/build && cd /opt/robotology-superbuild/build && \
    cmake .. \
        -DROBOTOLOGY_USES_ROS2=ON \
        -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON \
        -DYCM_INSTALL_DIR=/usr/share/YCM \
        -DYCM_DIR=/usr/share/YCM \
        -DCMAKE_PREFIX_PATH=/usr/share/YCM && \
    cmake --build . --config Release && \
    cmake --build . --target install

# Set up ROS 2 workspace
WORKDIR $WORKSPACE
RUN mkdir -p src && cd src && \
    git clone https://github.com/your/control-package.git && \
    cd .. && \
    colcon build

# Add environment setup to bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /opt/robotology-superbuild/build/install/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc && \
    echo "export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH" >> ~/.bashrc

# Expose necessary ports
EXPOSE 11311 10000 8888

# Default command to start Jupyter Notebook and bash
CMD ["bash", "-c", "jupyter notebook --ip=0.0.0.0 --port=8888 --allow-root & bash"]


