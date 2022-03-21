//
// Created by shuoy on 11/7/21.
//

#ifndef A1_CPP_HARDWAREA1ROS_H
#define A1_CPP_HARDWAREA1ROS_H

// std
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>

// control parameters
#include "A1Params.h"
#include "A1CtrlStates.h"
#include "A1RobotControl.h"
#include "A1BasicEKF.h"
#include "legKinematics/A1Kinematics.h"
#include "utils/Utils.h"

// a1 hardware
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#define FOOT_FILTER_WINDOW_SIZE 5

// one of the most important thing: A1 hardware return info with FR, FL, RR, RL order and receives info in this order
// we need to take of this order in this function
class HardwareA1ROS {
public:
    HardwareA1ROS(ros::NodeHandle &_nh);

    ~HardwareA1ROS() {
        destruct = true;
        thread_.join();
    }

    bool update_foot_forces_grf(double dt);

    bool main_update(double t, double dt);

    bool send_cmd();

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

private:
    ros::NodeHandle nh;
    ros::Publisher pub_joint_cmd;
    ros::Publisher pub_joint_angle;
    ros::Publisher pub_imu;
    sensor_msgs::JointState joint_foot_msg;
    sensor_msgs::Imu imu_msg;
    ros::Subscriber sub_joy_msg;

    // debug estimated position
    ros::Publisher pub_estimated_pose;

    // a1 hardware
    UNITREE_LEGGED_SDK::UDP udp;
    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::LowState state = {0};
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};
    // a1 hardware reading thread
    std::thread thread_;
    bool destruct = false;

    void udp_init_send();

    void receive_low_state();

    // a1 hardware switch foot order
    Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

    // a1 hardware foot force filter
    Eigen::Matrix<double, NUM_LEG, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
    Eigen::Matrix<int, NUM_LEG, 1> foot_force_filters_idx;
    Eigen::Matrix<double, NUM_LEG, 1> foot_force_filters_sum;


    // joystic command
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;
    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.12;

    //  0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;

    // following cade is also in VILEOM
    // add leg kinematics
    // the leg kinematics is relative to body frame, which is the center of the robot
    // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
    Eigen::Vector3d p_br;
    Eigen::Matrix3d R_br;
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};
    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;
    A1Kinematics a1_kin;
    // variables related to control and estimation
    A1CtrlStates a1_ctrl_states;
    A1RobotControl _root_control;
    A1BasicEKF a1_estimate;
};

#endif //A1_CPP_HARDWAREA1ROS_H
