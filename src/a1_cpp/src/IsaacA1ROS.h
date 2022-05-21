//
// Created by shuoy on 10/17/21.
//

#ifndef A1_CPP_ISAACA1ROS_H
#define A1_CPP_ISAACA1ROS_H

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

class IsaacA1ROS {
public:
    IsaacA1ROS(ros::NodeHandle &_nh);

    bool update_foot_forces_grf(double dt);

    bool main_update(double t, double dt);

    bool send_cmd();

    // callback functions
    void gt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_data);

    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu);

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &a1_state);

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

private:
    ros::NodeHandle nh;
    ros::Publisher pub_joint_cmd;
    ros::Subscriber sub_gt_pose_msg;
    ros::Subscriber sub_imu_msg;
    ros::Subscriber sub_joint_foot_msg;
    ros::Subscriber sub_joy_msg;

    // debug estimated position
    ros::Publisher pub_estimated_pose;

    // joystic command
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;

    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.32;

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
    
    // filters
    MovingWindowFilter acc_x;
    MovingWindowFilter acc_y;
    MovingWindowFilter acc_z;
    MovingWindowFilter gyro_x;
    MovingWindowFilter gyro_y;
    MovingWindowFilter gyro_z;
};

#endif //A1_CPP_ISAACA1ROS_H
