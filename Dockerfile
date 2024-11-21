# Use the Robotology base image with Gazebo
FROM robotology/robotology-tdd:gazebo11master

# Install Kitware key to avoid NO_PUBKEY error
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1A127079A92F09ED

# Install additional dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11vnc xvfb supervisor novnc websockify cmake build-essential \
    git wget software-properties-common && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Add Robotology PPA and install YARP dependencies
RUN add-apt-repository ppa:robotology/ppa && \
    apt-get update && apt-get install -y yarp && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up VNC and directories
RUN mkdir -p /root/.vnc && x11vnc -storepasswd "vncpassword" /root/.vnc/passwd

# Clone and build the ErgoCub Gazebo simulations repository
RUN mkdir -p /workspace && \
    cd /workspace && \
    git clone https://github.com/icub-tech-iit/ergocub-gazebo-simulations.git && \
    cd ergocub-gazebo-simulations && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# Clone and build YARP and dependencies
RUN cd /workspace && \
    git clone https://github.com/robotology/ycm.git && \
    cd ycm && mkdir build && cd build && \
    cmake .. && make -j4 && make install && \
    cd /workspace && \
    git clone https://github.com/robotology/yarp.git && \
    cd yarp && mkdir build && cd build && \
    cmake -DYCM_DIR=/usr/local/share/YCM .. && \
    make -j4 && make install

# Clone and build Gazebo YARP Plugins
RUN cd /workspace && \
    git clone https://github.com/robotology/gazebo-yarp-plugins.git && \
    cd gazebo-yarp-plugins && \
    rm -rf build && mkdir build && cd build && \
    cmake .. -DYCM_DIR=/usr/local/share/YCM -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j4 VERBOSE=1 && \
    make install

ENV GAZEBO_MODEL_PATH=/workspace/ergocub-gazebo-simulations:/workspace/ergocub-gazebo-simulations/conf_stickBot
ENV ERGOCUB_GAZEBO_PATH=/workspace/ergocub-gazebo-simulations
ENV GAZEBO_MODEL_PATH=$ERGOCUB_GAZEBO_PATH/models
ENV GAZEBO_PLUGIN_PATH=/workspace/gazebo-yarp-plugins/build:/usr/local/lib/gazebo/plugins
ENV LD_LIBRARY_PATH=/usr/local/lib:/workspace/gazebo-yarp-plugins/build
ENV YARP_DATA_DIRS=/usr/local/share/yarp:$ERGOCUB_GAZEBO_PATH
ENV PATH=$PATH:/usr/local/bin

# Copy supervisord configuration
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

# Expose ports
EXPOSE 8080 5900 10000

# Start supervisord
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf"]



















