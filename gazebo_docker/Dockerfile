# we build this docker based on a docker with ros already installed
FROM osrf/ros:melodic-desktop-full

LABEL maintainer="shuoyang@andrew.cmu.edu"
ENV REFRESH_AT 2022-03-22

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# install necessary dependencies 
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      vim \
      libatlas-base-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python-catkin-tools \
      python3-matplotlib \
      gfortran \
      autoconf \
      coinor-libipopt-dev \
      libgfortran3 \
      curl \
      libopenmpi-dev \
      apt-utils \
      software-properties-common \
      build-essential \
      libssl-dev \
      wget \
      libtool \
      cmake

# install python 3.6 
RUN apt-get update && \
    apt-get install -y python3.6
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.6 1
RUN curl https://bootstrap.pypa.io/pip/3.6/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN rm get-pip.py

# change timezone (this is very important otherwise many ROS topic time will be strange)
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# make unitree workspace
ENV SUPPORT_WS=/root/support_files
ENV UNITREE_WS=/root/unitree_ws
RUN mkdir -p $SUPPORT_WS 
RUN mkdir -p $UNITREE_WS/src
WORKDIR $UNITREE_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin init;"

# install unitree sdk dependencies
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/lcm-proj/lcm.git && \
    cd ${SUPPORT_WS}/lcm && \
    git checkout tags/v1.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 && \
    make install 

# notice we must use v3.2 
WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/unitree_legged_sdk.git && \
    cd ${SUPPORT_WS}/unitree_legged_sdk && git checkout v3.2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12 

WORKDIR $SUPPORT_WS
RUN git clone https://github.com/unitreerobotics/aliengo_sdk.git && \
    cd ${SUPPORT_WS}/aliengo_sdk && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j12     

# install necessary dependencies 
RUN   if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-ros-control \
      ros-${ROS_DISTRO}-gazebo-ros \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-ros-controllers \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-plotjuggler-ros

WORKDIR $UNITREE_WS/src
RUN git clone https://github.com/ShuoYangRobotics/unitree_ros.git
WORKDIR $UNITREE_WS

ENV UNITREE_SDK_VERSION=3_2
ENV UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk
ENV ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk
ENV UNITREE_PLATFORM=amd64
RUN echo "#unitree config" >> ~/.bashrc
RUN echo "export export UNITREE_SDK_VERSION=3_2" >> ~/.bashrc
RUN echo "export UNITREE_LEGGED_SDK_PATH=${SUPPORT_WS}/unitree_legged_sdk" >> ~/.bashrc
RUN echo "export ALIENGO_SDK_PATH=${SUPPORT_WS}/aliengo_sdk" >> ~/.bashrc
RUN echo "export UNITREE_PLATFORM=\"amd64\"" >> ~/.bashrc
RUN echo "source ${UNITREE_WS}/devel/setup.bash" >> ~/.bashrc

# compile unitree ros 
RUN ls $UNITREE_WS/src/unitree_ros
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build unitree_legged_msgs; source ${UNITREE_WS}/devel/setup.bash; catkin build"


RUN echo "export ROS_MASTER_URI=http://localhost:11311;export ROS_IP=localhost;export ROS_HOSTNAME=localhost" >> ~/.bashrc

# To use rosparam load yaml files
RUN pip install pyyaml

# default use python 2.7, or type this command when docker starts 
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2.7 3