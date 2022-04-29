FROM osrf/ros:melodic-desktop-full

ENV LANG en_US.UTF-8

# TO fix the GPG key bug: https://answers.ros.org/question/379190/apt-update-signatures-were-invalid-f42ed6fbab17c654/
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update && apt-get install -y \
        tmux \
        zsh \
        curl \
        wget \
        vim \
        sudo \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        mesa-utils \
        unzip \
        python-catkin-tools \
        ros-melodic-hector-gazebo-plugins \
        python3-pip \
        # ros-melodic-dynamixel-sdk \
        build-essential \
        libboost-all-dev \
        swig \
        libeigen3-dev \
        python-numpy \
        pybind11-dev \
        python-dev \
        python3-dev \
        ros-melodic-soem \
        gcc-multilib \
        g++-multilib \
        libgtest-dev \
        && rm -rf /var/likb/apt/lists/*

# Install cmake 20 to resolve several problems with main granch of Romelautilities
RUN cd ~/ && mkdir download && cd download &&                                                  \
    wget 'https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz' &&   \
    tar -zxvf 'cmake-3.20.2.tar.gz' &&                                                         \
    cd 'cmake-3.20.2' && ./bootstrap --prefix=$HOME/cmake-install && make && make install

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && apt-get update && apt-get install -q -y && \
    source /opt/ros/melodic/setup.bash && \
    mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    catkin_init_workspace

# Eigen3
RUN cd /usr/include/ && git clone https://gitlab.com/libeigen/eigen.git
ENV CMAKE_PREFIX_PATH="/usr/include/:${CMAKE_PREFIX_PATH}"

WORKDIR "~/"
COPY requirements.txt /root
RUN pip3 install -r ~/requirements.txt
COPY PySHMXtreme/ /root/catkin_ws/pySiLVIA_lib/PySHMXtreme/
RUN pip3 install -e /root/catkin_ws/pySiLVIA_lib/PySHMXtreme

RUN echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Install driver for FT sensor
RUN cd ~/catkin_ws/src && git clone https://gitlab.com/botasys/bota_driver.git && \
    cd bota_driver && git checkout dda7f5fbe530a43c37c633d88baee7706e6dc349 && cd ~/catkin_ws && rosdep update && \
    rosdep install --from-path src --ignore-src -y
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN echo "log4j.logger.ros=WARNING" > /opt/ros/melodic/share/ros/config/rosconsole.config && echo "log4j.logger.ros=FATAL" >>  /opt/ros/melodic/share/ros/config/rosconsole.config


