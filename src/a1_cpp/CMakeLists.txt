cmake_minimum_required(VERSION 3.10.1)
project(a1_cpp)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#check environmental variables
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        unitree_legged_msgs
        )

find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES rc_sim_oub
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

# to find osqp.h
include_directories(/usr/local/include/osqp)
find_package(OsqpEigen REQUIRED)

# add unitree hardware library, these flags must be correctly set
include_directories(
        $ENV{UNITREE_LEGGED_SDK_PATH}/include
)

link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)

# Declare cpp libraries
# common robot controller every type of robots need
add_library(a1_lib
        src/legKinematics/A1Kinematics.h
        src/legKinematics/A1Kinematics.cpp
        src/A1Params.h
        src/A1CtrlStates.h
        src/utils/Utils.cpp
        src/utils/Utils.h
        src/A1RobotControl.cpp
        src/A1RobotControl.h
        src/A1BasicEKF.cpp
        src/A1BasicEKF.h
        src/ConvexMpc.cpp
        src/ConvexMpc.h
        )
target_link_libraries(a1_lib ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

# lib of different types of robots
add_library(gazebo_a1_lib
        src/GazeboA1ROS.h
        src/GazeboA1ROS.cpp
        )

add_library(isaac_a1_lib
        src/IsaacA1ROS.h
        src/IsaacA1ROS.cpp
        )

add_library(hardware_a1_lib
        src/HardwareA1ROS.h
        src/HardwareA1ROS.cpp
        )

# all robots depends on the a1_lib
target_link_libraries(gazebo_a1_lib a1_lib)
target_link_libraries(isaac_a1_lib a1_lib)
target_link_libraries(hardware_a1_lib a1_lib ${EXTRA_LIBS})

# Declare a cpp executable for isaac robot
add_executable(isaac_a1_ctrl src/MainIsaac.cpp)
target_link_libraries(isaac_a1_ctrl
        isaac_a1_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        )

# Declare a cpp executable for gazebo robot
add_executable(gazebo_a1_ctrl src/MainGazebo.cpp)
target_link_libraries(gazebo_a1_ctrl
        gazebo_a1_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        )

# Declare a cpp executable for hardware robot
add_executable(hardware_a1_ctrl src/MainHardware.cpp)
target_link_libraries(hardware_a1_ctrl
        hardware_a1_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        )

# tests
# Declare a cpp executable

add_executable(test_rotation src/test/test_rotation.cpp
        src/utils/Utils.cpp
        src/utils/Utils.h
        )

add_executable(test_bezier src/test/test_bezier.cpp
        src/utils/Utils.cpp
        src/utils/Utils.h
        )

add_executable(test_mpc src/test/test_mpc.cpp)
target_link_libraries(test_mpc
        a1_lib
        ${catkin_LIBRARIES}
        OsqpEigen::OsqpEigen
        )

add_executable(test_unitree_legged_msgs src/test/test_unitree_msgs.cpp)
target_link_libraries(test_unitree_legged_msgs
        ${catkin_LIBRARIES}
        )
