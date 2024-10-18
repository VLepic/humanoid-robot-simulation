# Vybereme základní obraz pro ROS Noetic na Ubuntu 20.04
FROM ros:noetic-ros-base-focal

# Nastavení proměnných prostředí
ENV DEBIAN_FRONTEND=noninteractive
ENV USER=root

# Aktualizace balíčků a instalace potřebných nástrojů
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    git \
    wget \
    software-properties-common \
    x11vnc \
    xvfb \
    xterm \
    xfce4 \
    supervisor \
    novnc \
    websockify \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins

# Instalace novější verze CMake (3.20 nebo vyšší)
RUN apt-get remove -y cmake && \
    wget https://github.com/Kitware/CMake/releases/download/v3.22.0/cmake-3.22.0-linux-x86_64.sh && \
    mkdir /opt/cmake && \
    sh cmake-3.22.0-linux-x86_64.sh --prefix=/opt/cmake --skip-license && \
    ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake

# Instalace YARP závislostí
RUN apt-get update && \
    apt-get install -y \
    libace-dev \
    libgsl-dev \
    libeigen3-dev \
    libopencv-dev \
    libsqlite3-dev \
    libedit-dev \
    libtinyxml-dev \
    libzmq3-dev

# Klonování a instalace YCM (YARP CMake Modules)
RUN mkdir -p /opt/ycm && cd /opt/ycm && \
    git clone https://github.com/robotology/ycm.git && \
    cd ycm && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Klonování zdrojového kódu YARP
RUN mkdir -p /opt/yarp && cd /opt/yarp && \
    git clone https://github.com/robotology/yarp.git && \
    cd yarp && \
    git checkout master

# Kompilace a instalace YARP
RUN cd /opt/yarp/yarp && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Nastavení YARP proměnných prostředí
ENV YARP_DIR=/opt/yarp/yarp/build
ENV PATH=$PATH:$YARP_DIR/bin
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/yarp/yarp/build/lib

# Instalace ROS balíčků pro práci s Gazebo
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-gazebo-ros-pkgs

# Klonování ergocub-gazebo-simulations
RUN mkdir -p /opt/ergocub && cd /opt/ergocub && \
    git clone https://github.com/icub-tech-iit/ergocub-gazebo-simulations.git


# Nastavení proměnných prostředí pro Ergocub simulace
ENV GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/opt/ergocub/ergocub-gazebo-simulations/models
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:/opt/ergocub/ergocub-gazebo-simulations

# Instalace závislostí pro simulace v Gazebo
RUN apt-get update && \
    apt-get install -y \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    ros-noetic-joint-trajectory-controller

# Instalace gazebo-yarp-plugins
RUN mkdir -p /opt/gazebo-yarp-plugins && cd /opt/gazebo-yarp-plugins && \
    git clone https://github.com/robotology/gazebo-yarp-plugins.git

RUN cd /opt/gazebo-yarp-plugins/gazebo-yarp-plugins && \
    mkdir build && cd build && \
    cmake .. && \
    make -j2 && \
    make install

# Kompilace ROS workspace s ergocub-gazebo-simulations
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    ln -s /opt/ergocub/ergocub-gazebo-simulations . && \
    cd /root/catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Nastavení VNC a noVNC
RUN mkdir -p /root/.vnc && \
    x11vnc -storepasswd "vncpassword" /root/.vnc/passwd

COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Exponujeme porty pro VNC a noVNC
EXPOSE 8080 5900

# Spouštěcí příkaz
CMD ["/usr/bin/supervisord"]




