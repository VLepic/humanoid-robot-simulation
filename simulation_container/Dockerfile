# Base image
FROM kasmweb/core-cuda-focal:1.16.1

# Set environment variables for the container
ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm

ARG ROBOTOLOGY_SUPERBUILD_VERSION=2024.11.2
ARG ERGOCUB_SOFTWARE_VERSION=0.7.6
ARG CMAKE_VERSION=3.31.5
ARG ROS2_DISTRO=galactic


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
    rm -rf /var/lib/apt/lists/*

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


RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    git \
    qt5-default \
    qtbase5-dev \
    qtbase5-dev-tools \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libx11-xcb1 \
    && rm -rf /var/lib/apt/lists/* && \
    pip3 install PyQt5 && \
    pip install transforms3d

# Klonování aplikace (větev PyQt5)
RUN git clone --branch GazeboClassicPyQt5 https://github.com/VLepic/GazeboWalledWorldDesigner /opt/GazeboWalledWorldDesigner

RUN mkdir ${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/ergoCub/robots/ergocub_modified

RUN apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    sudo apt update && \
    apt install -y ros-${ROS2_DISTRO}-gazebo-ros-pkgs

# Install SLAM toolbox
RUN . &&\
    apt install -y \
    ros-galactic-slam-toolbox \
    ros-galactic-tf2-tools \
    ros-galactic-nav2-map-server \
    ros-galactic-xacro \
    python3-vcstool

#Build ergocub ros2 package
COPY ./simulation_container/ergocub_slam_bridge /opt/ros_ws/src/ergocub_slam_bridge
WORKDIR /opt/ros_ws
RUN . /opt/ros/galactic/setup.sh && \
    colcon build --merge-install --install-base /opt/ros/galactic

#Installing rviz2
RUN apt-get update && apt-get install -y \
    ros-galactic-rviz2 \
    libogre-1.12-dev \
    ros-galactic-rqt \
    ros-galactic-nav2-bringup \
    libcanberra-gtk-module &&\
    rm -rf /var/lib/apt/lists/*
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/ros/galactic/opt/rviz_ogre_vendor/lib
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/ros/galactic/opt/yaml_cpp_vendor/lib


# YARP Server Desktop Entry
RUN echo '#!/usr/bin/env bash\nyarpserver' > /usr/local/bin/start_yarpserver.sh && \
    chmod +x /usr/local/bin/start_yarpserver.sh && \
    echo "[Desktop Entry]\n\
    Version=1.0\n\
    Type=Application\n\
    Name=YARP Server\n\
    Exec=/usr/local/bin/start_yarpserver.sh\n\
    Icon=utilities-terminal\n\
    Terminal=true\n\
    Categories=Development;" > /usr/share/applications/yarpserver.desktop && \
    chmod 644 /usr/share/applications/yarpserver.desktop

# Gazebo Walled World Designer
RUN echo '#!/usr/bin/env bash\npython3 /opt/GazeboWalledWorldDesigner/main.py' > /usr/local/bin/start_GazeboWalledWorldDesigner.sh && \
    chmod +x /usr/local/bin/start_GazeboWalledWorldDesigner.sh && \
    echo "[Desktop Entry]\n\
    Version=1.0\n\
    Type=Application\n\
    Name=Gazebo Walled World Designer\n\
    Exec=/usr/local/bin/start_GazeboWalledWorldDesigner.sh\n\
    Icon=utilities-terminal\n\
    Terminal=true\n\
    Categories=Development;" > /usr/share/applications/GazeboWalledWorldDesigner.desktop && \
    chmod 644 /usr/share/applications/GazeboWalledWorldDesigner.desktop

# Gazebo Desktop Entry
RUN echo '#!/usr/bin/env bash\ngazebo --verbose /opt/gazebo_worlds/generated.world' > /usr/local/bin/start_gazebo.sh && \
    chmod +x /usr/local/bin/start_gazebo.sh && \
    echo "[Desktop Entry]\n\
    Version=1.0\n\
    Type=Application\n\
    Name=Gazebo\n\
    Exec=/usr/local/bin/start_gazebo.sh\n\
    Icon=utilities-terminal\n\
    Terminal=true\n\
    Categories=Development;" > /usr/share/applications/gazebo.desktop && \
    chmod 644 /usr/share/applications/gazebo.desktop

# ROS2 Launch for slam_with_yarp.launch.py
RUN echo '#!/usr/bin/env bash\nsource /opt/ros/galactic/setup.bash && source /opt/ros/galactic/local_setup.bash && ros2 launch ergocub_slam_bridge slam_with_yarp.launch.py' > /usr/local/bin/start_slam_with_yarp.sh && \
    chmod +x /usr/local/bin/start_slam_with_yarp.sh && \
    echo "[Desktop Entry]\n\
    Version=1.0\n\
    Type=Application\n\
    Name=SLAM with YARP\n\
    Exec=/usr/local/bin/start_slam_with_yarp.sh\n\
    Icon=utilities-terminal\n\
    Terminal=true\n\
    Categories=Development;" > /usr/share/applications/slam_with_yarp.desktop && \
    chmod 644 /usr/share/applications/slam_with_yarp.desktop

# Move selected .desktop entries to Desktop safely
RUN mkdir -p /home/kasm-user/Desktop && \
    install -m 644 /usr/share/applications/gazebo.desktop /home/kasm-user/Desktop/ && \
    install -m 644 /usr/share/applications/GazeboWalledWorldDesigner.desktop /home/kasm-user/Desktop/ && \
    install -m 644 /usr/share/applications/yarpserver.desktop /home/kasm-user/Desktop/ && \
    install -m 644 /usr/share/applications/slam_with_yarp.desktop /home/kasm-user/Desktop/ && \
    chown -R kasm-user:kasm-user /home/kasm-user/Desktop

COPY ./simulation_container/wholebodycfg/wbd_ecub_sim.xml /opt/robotology-superbuild/src/ergocub-software/urdf/ergoCub/conf/estimators/
COPY ./simulation_container/wholebodycfg/dcm_walking_with_joypad.ini /opt/robotology-superbuild/build-base/install/share/ICUBcontrib/robots/ergoCubGazeboV1/
# Set the default command to run the YARP server
CMD ["/usr/bin/supervisord && . /opt/ros/${ROS2_DISTRO}/setup.sh"]