# we build this docker based on a docker with ros already installed
FROM ros:melodic-perception
# fixed a key issue using the method mentioned in https://github.com/PRBonn/bonnetal/commit/0ab883b8356954c3e57a07a73c30bbac8f035a05
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
  apt update 

LABEL maintainer="shuoyang@andrew.cmu.edu"
ENV REFRESH_AT 2022-03-20

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
      libssl-dev

# install python 3.6 
RUN apt-get update && \
    apt-get install -y python3.6
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.6 1
RUN curl https://bootstrap.pypa.io/pip/3.6/get-pip.py -o get-pip.py
RUN python get-pip.py
RUN rm get-pip.py
RUN pip --version
RUN pip install pybullet==3.1.7
RUN pip install tensorflow==1.15.4
RUN pip install numpy==1.17.3
RUN pip install scipy


# install cmake 3.20 
WORKDIR /tmp 
RUN apt-get -y install wget libtool
RUN wget https://github.com/Kitware/CMake/releases/download/v3.20.2/cmake-3.20.2.tar.gz
RUN tar -zxvf cmake-3.20.2.tar.gz
RUN ls
WORKDIR /tmp/cmake-3.20.2
RUN ./bootstrap
RUN make
RUN make install

# install motion_imitation
# WORKDIR /root
# RUN git clone https://github.com/paulyang1990/motion_imitation.git 
# WORKDIR /root/motion_imitation
# RUN pwd
# RUN pip install -r requirements.txt
# # remove it because we will use mount file
# WORKDIR /root
# RUN rm -rf /root/motion_imitation  
# # this is for Tiny Differentiable Simulator
# RUN pip install meshcat

# change timezone (this is very important otherwise many ROS topic time will be strange)
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# add OSQP 
# follow https://osqp.org/docs/get_started/sources.html#build-from-sources to install OSQP from sources
WORKDIR /tmp
RUN git clone --recursive https://github.com/oxfordcontrol/osqp
WORKDIR /tmp/osqp
RUN mkdir build 
WORKDIR /tmp/osqp/build
RUN cmake -G "Unix Makefiles" ..
RUN cmake --build .
RUN cmake --build . --target install

# add OSQP-python
RUN pip install osqp
RUN apt-get install -y ipython

# add osqp-eigen
WORKDIR /tmp
RUN git clone https://github.com/robotology/osqp-eigen.git
WORKDIR /tmp/osqp-eigen
RUN mkdir build 
WORKDIR /tmp/osqp-eigen/build
RUN cmake ../
RUN make
RUN make install
# -- Installing: /usr/local/lib/libOsqpEigen.so.0.6.3
# -- Set runtime path of "/usr/local/lib/libOsqpEigen.so.0.6.3" to "$ORIGIN/:$ORIGIN/../lib:/usr/local/lib"
# -- Installing: /usr/local/lib/libOsqpEigen.so
# -- Installing: /usr/local/include/OsqpEigen/OsqpEigen.h
# -- Installing: /usr/local/include/OsqpEigen/Constants.hpp
# -- Installing: /usr/local/include/OsqpEigen/SparseMatrixHelper.hpp
# -- Installing: /usr/local/include/OsqpEigen/SparseMatrixHelper.tpp
# -- Installing: /usr/local/include/OsqpEigen/Data.hpp
# -- Installing: /usr/local/include/OsqpEigen/Data.tpp
# -- Installing: /usr/local/include/OsqpEigen/Settings.hpp
# -- Installing: /usr/local/include/OsqpEigen/Solver.hpp
# -- Installing: /usr/local/include/OsqpEigen/Solver.tpp
# -- Installing: /usr/local/include/OsqpEigen/Debug.hpp
# -- Installing: /usr/local/lib/cmake/OsqpEigen/OsqpEigenConfigVersion.cmake
# -- Installing: /usr/local/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake
# -- Installing: /usr/local/lib/cmake/OsqpEigen/OsqpEigenTargets.cmake
# -- Installing: /usr/local/lib/cmake/OsqpEigen/OsqpEigenTargets-release.cmake

# add pytorch for cpu only system 
# RUN pip install torch==1.8.1+cpu torchvision==0.9.1+cpu torchaudio==0.8.1 -f https://download.pytorch.org/whl/torch_stable.html
# RUN pip install jupyter gym

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
      ros-${ROS_DISTRO}-robot-state-publisher

WORKDIR $UNITREE_WS/src
RUN git clone https://github.com/ShuoYangRobotics/unitree_ros.git
WORKDIR $UNITREE_WS
# there are some non ascii code in this file that prevents docker from catkin build the file
# RUN perl -pi -e 's/[^[:ascii:]]//g' $UNITREE_WS/src/unitree_ros/unitree_legged_msgs/msg/MotorState.msg 
# RUN perl -pi -e 's/[^[:ascii:]]//g' $UNITREE_WS/src/unitree_ros/unitree_legged_msgs/msg/MotorState.msg 
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
# RUN echo "export ROS_MASTER_URI=http://192.168.123.2:11311;export ROS_IP=192.168.123.2;export ROS_HOSTNAME=192.168.123.2" >> ~/.bashrc
# compile just unitree ros unitree_legged_msgs
RUN ls $UNITREE_WS/src/unitree_ros
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin build unitree_legged_msgs;"


# https://github.com/JetBrains/clion-remote/blob/master/Dockerfile.remote-cpp-env
# ssh credentials (test user):
#   user@password 
RUN DEBIAN_FRONTEND="noninteractive" apt-get update && apt-get -y install tzdata

RUN apt-get update \
  && apt-get install -y ssh \
      build-essential \
      gcc \
      g++ \
      gdb \
      clang \
      make \
      ninja-build \
      cmake \
      autoconf \
      automake \
      locales-all \
      dos2unix \
      rsync \
      tar \
      python \
      python-rospkg \
  && apt-get clean

# https://www.allaban.me/posts/2020/08/ros2-setup-ide-docker/
#https://stackoverflow.com/questions/62417777/start-docker-container-with-host-network-while-maintaining-the-ability-to-ssh-in
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
    echo 'Port 2233'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

# Change the password 'password' to something more secure
RUN useradd -m user && yes password | passwd user
RUN echo 'root:password' | chpasswd

CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]

RUN echo "export ROS_MASTER_URI=http://localhost:11311;export ROS_IP=localhost;export ROS_HOSTNAME=localhost" >> ~/.bashrc

# To use rosparam load yaml files
RUN pip install pyyaml

# default use python 2.7, or type this command when docker starts 
RUN update-alternatives --install /usr/bin/python python /usr/bin/python2.7 3

### How to build this docker 
# sudo docker build --build-arg -t="a1_ros_ctrl_image" .

### run docker as a ssh daemon
# sudo docker run -d --network host -v /home/shuoy/research/A1_Ctrl:/root/A1_Ctrl  --name a1_ros_ctrl_docker a1_ros_ctrl_image
# docker start xx
# ssh root@localhost -p2233

# config clion environment:
# File -> settings -> Build, Execution, Deployment -> Toolchains  add remote host
# File -> settings -> Build, Execution, Deployment -> CMake add "Cmake Options": 
# -DPYTHON_EXECUTABLE=/usr/bin/python2

# File -> settings -> Build, Execution, Deployment -> CMake add "Environment":
# LD_LIBRARY_PATH=/root/unitree_ws/devel/lib:/opt/ros/melodic/lib;ROS_ETC_DIR=/opt/ros/melodic/etc/ros;UNITREE_PLATFORM=amd64;ROS_ROOT=/opt/ros/melodic/share/ros;ROS_MASTER_URI=http://localhost:11311;ROS_VERSION=1;ROS_PYTHON_VERSION=2;ROS_IP=127.0.0.1;PYTHONPATH=/root/unitree_ws/devel/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages;ROS_PACKAGE_PATH=/root/unitree_ws/src/unitree_ros/unitree_legged_msgs:/opt/ros/melodic/share;ROSLISP_PACKAGE_DIRECTORIES=/root/unitree_ws/devel/share/common-lisp;ALIENGO_SDK_PATH=/root/support_files/aliengo_sdk;UNITREE_SDK_VERSION=3_2;PATH=/opt/ros/melodic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin;PKG_CONFIG_PATH=/root/unitree_ws/devel/lib/pkgconfig:/opt/ros/melodic/lib/pkgconfig;UNITREE_LEGGED_SDK_PATH=/root/support_files/unitree_legged_sdk;ROS_HOSTNAME=localhost;ROS_DISTRO=melodic;CMAKE_PREFIX_PATH=/root/unitree_ws/devel:/opt/ros/melodic
# (do not include host system environmentvariables)

# File -> settings -> Build, Execution, Deployment -> CMake Toolchain use "Remote host"

# clion will compile executable file into /tmp/tmp.XXXXX/cmake-build-debug/devel/lib
# while in the docker, if run "catkin build" at /root/A1_Ctrl, the executable will be at /root/A1_Ctrl/devel/lib/a1_cpp
# either way we have the executable to control the robot 
