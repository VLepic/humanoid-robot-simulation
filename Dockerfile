# Use the Robotology base image with Gazebo
FROM robotology/robotology-tdd:gazebo11master

# Install Kitware key to avoid NO_PUBKEY error
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1A127079A92F09ED

# Install additional dependencies for noVNC and X11 server
RUN apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    supervisor \
    novnc \
    websockify && \
    apt-get clean

# Set up directories and password for VNC
RUN mkdir -p /root/.vnc && \
    x11vnc -storepasswd "vncpassword" /root/.vnc/passwd

# Create the workspace directory and clone the ergocub-gazebo-simulations repository
RUN mkdir -p /workspace && \
    cd /workspace && \
    git clone https://github.com/icub-tech-iit/ergocub-gazebo-simulations.git && \
    cd ergocub-gazebo-simulations && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# Set environment variables for Gazebo model and plugin paths
ENV ERGOCUB_GAZEBO_PATH=/workspace/ergocub-gazebo-simulations
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$ERGOCUB_GAZEBO_PATH/models
ENV GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$ERGOCUB_GAZEBO_PATH/build

# Copy the supervisord configuration
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Expose ports for noVNC and VNC
EXPOSE 8080 5900

# Start supervisord to manage processes
CMD ["/usr/bin/supervisord"]










