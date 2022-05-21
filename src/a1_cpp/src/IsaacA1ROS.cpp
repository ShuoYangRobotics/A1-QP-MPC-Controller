//
// Created by shuoy on 10/17/21.
//

#include "IsaacA1ROS.h"

// constructor
IsaacA1ROS::IsaacA1ROS(ros::NodeHandle &_nh) {

    nh = _nh;
    // ROS publisher
    pub_joint_cmd = nh.advertise<sensor_msgs::JointState>("/isaac_a1/joint_torque_cmd", 100);
    // ROS register callback, call backs directly modify variables in A1CtrlStates
    sub_gt_pose_msg = nh.subscribe(
            "/isaac_a1/gt_body_pose", 100, &IsaacA1ROS::gt_pose_callback, this);
    sub_imu_msg = nh.subscribe(
            "/isaac_a1/imu_data", 100, &IsaacA1ROS::imu_callback, this);
    sub_joint_foot_msg = nh.subscribe(
            "/isaac_a1/joint_foot", 100, &IsaacA1ROS::joint_state_callback, this);
    sub_joy_msg = nh.subscribe("/joy", 1000, &IsaacA1ROS::joy_callback, this);

    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/isaac_a1/estimation_body_pose", 100);

    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    joy_cmd_exit = false;

    _root_control = A1RobotControl(nh);
    a1_ctrl_states.reset();
    a1_ctrl_states.resetFromROSParam(nh);

    // init leg kinematics
    // set leg kinematics related parameters
    // body_to_a1_body
    p_br = Eigen::Vector3d(-0.2293, 0.0, -0.067);
    R_br = Eigen::Matrix3d::Identity();
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.22;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }
    acc_x = MovingWindowFilter(5);
    acc_y = MovingWindowFilter(5);
    acc_z = MovingWindowFilter(5);
    gyro_x = MovingWindowFilter(5);
    gyro_y = MovingWindowFilter(5);
    gyro_z = MovingWindowFilter(5);
}

bool IsaacA1ROS::update_foot_forces_grf(double dt) {
    a1_ctrl_states.foot_forces_grf = _root_control.compute_grf(a1_ctrl_states, dt);
    return true;
}

bool IsaacA1ROS::main_update(double t, double dt) {
    if (joy_cmd_exit) {
        return false;
    }

    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into a1_ctrl_states
    joy_cmd_body_height += joy_cmd_velz * dt;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }

    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;

    if (joy_cmd_ctrl_state_change_request) {
        // toggle joy_cmd_ctrl_state
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2; //TODO: how to toggle more states?
        joy_cmd_ctrl_state_change_request = false; //erase this change request;
    }

    // root_lin_vel_d is in robot frame
    a1_ctrl_states.root_lin_vel_d[0] = joy_cmd_velx;
    a1_ctrl_states.root_lin_vel_d[1] = joy_cmd_vely;

    // root_ang_vel_d is in robot frame
    a1_ctrl_states.root_ang_vel_d[0] = joy_cmd_roll_rate;
    a1_ctrl_states.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    a1_ctrl_states.root_ang_vel_d[2] = joy_cmd_yaw_rate;
    a1_ctrl_states.root_euler_d[0] += joy_cmd_roll_rate * dt;
    a1_ctrl_states.root_euler_d[1] += joy_cmd_pitch_rate * dt;
    a1_ctrl_states.root_euler_d[2] += joy_cmd_yaw_rate * dt;
    a1_ctrl_states.root_pos_d[2] = joy_cmd_body_height;

    // determine movement mode
    if (joy_cmd_ctrl_state == 1) {
        // in walking mode, in this mode the robot should execute gait
        a1_ctrl_states.movement_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        a1_ctrl_states.movement_mode = 0;
        a1_ctrl_states.root_pos_d.segment<2>(0) = a1_ctrl_states.root_pos.segment<2>(0);
        a1_ctrl_states.kp_linear(0) = a1_ctrl_states.kp_linear_lock_x;
        a1_ctrl_states.kp_linear(1) = a1_ctrl_states.kp_linear_lock_y;
    } else {
        a1_ctrl_states.movement_mode = 0;
    }

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (a1_ctrl_states.movement_mode == 1) {
        if (a1_ctrl_states.root_lin_vel_d.segment<2>(0).norm() > 0.05) {
            // has nonzero velocity, keep refreshing position target, but just xy
            a1_ctrl_states.root_pos_d.segment<2>(0) = a1_ctrl_states.root_pos.segment<2>(0);
            a1_ctrl_states.kp_linear.segment<2>(0).setZero();
        } else {
            a1_ctrl_states.kp_linear(0) = a1_ctrl_states.kp_linear_lock_x;
            a1_ctrl_states.kp_linear(1) = a1_ctrl_states.kp_linear_lock_y;
        }
    }

    _root_control.update_plan(a1_ctrl_states, dt);
    _root_control.generate_swing_legs_ctrl(a1_ctrl_states, dt);

    // state estimation
    if (!a1_estimate.is_inited()) {
        a1_estimate.init_state(a1_ctrl_states);
    } else {
        a1_estimate.update_estimation(a1_ctrl_states, dt);
    }

    nav_msgs::Odometry estimate_odom;
    estimate_odom.pose.pose.position.x = a1_ctrl_states.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = a1_ctrl_states.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = a1_ctrl_states.estimated_root_pos(2);

    // make sure root_lin_vel is in world frame
    estimate_odom.twist.twist.linear.x = a1_ctrl_states.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = a1_ctrl_states.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = a1_ctrl_states.estimated_root_vel(2);

    pub_estimated_pose.publish(estimate_odom);

    return true;
}

bool IsaacA1ROS::send_cmd() {
    _root_control.compute_joint_torques(a1_ctrl_states);

    // send control cmd to robot via ros topic
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.header.frame_id = "a1_body";
    joint_msg.effort.resize(NUM_DOF);

    for (int i = 0; i < NUM_DOF; i++) {
        joint_msg.effort[i] = a1_ctrl_states.joint_torques(i);
    }

    pub_joint_cmd.publish(joint_msg);

    return true;
}

// callback functions
void IsaacA1ROS::gt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_data) {
    // update
    a1_ctrl_states.root_quat = Eigen::Quaterniond(pose_data->pose.orientation.w,
                                                  pose_data->pose.orientation.x,
                                                  pose_data->pose.orientation.y,
                                                  pose_data->pose.orientation.z);
    a1_ctrl_states.root_pos << pose_data->pose.position.x,
            pose_data->pose.position.y,
            pose_data->pose.position.z;

    // calculate several useful variables
    // euler should be roll pitch yaw
    a1_ctrl_states.root_rot_mat = a1_ctrl_states.root_quat.toRotationMatrix();
//    a1_ctrl_states.root_euler = a1_ctrl_states.root_rot_mat.eulerAngles(0, 1, 2);
    a1_ctrl_states.root_euler = Utils::quat_to_euler(a1_ctrl_states.root_quat);
    double yaw_angle = a1_ctrl_states.root_euler[2];

    // make sure root_ang_vel is in world frame
    // a1_ctrl_states.root_ang_vel = a1_ctrl_states.root_rot_mat * a1_ctrl_states.imu_ang_vel;
    a1_ctrl_states.root_ang_vel = a1_ctrl_states.root_rot_mat * a1_ctrl_states.imu_ang_vel;

    a1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // FL, FR, RL, RR
    for (int i = 0; i < NUM_LEG; ++i) {
        a1_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = a1_kin.fk(
                a1_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        a1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = a1_kin.jac(
                a1_ctrl_states.joint_pos.segment<3>(3 * i),
                rho_opt_list[i], rho_fix_list[i]);
        Eigen::Matrix3d tmp_mtx = a1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
        Eigen::Vector3d tmp_vec = a1_ctrl_states.joint_vel.segment<3>(3 * i);
        a1_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

        a1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        a1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

        a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) = a1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + a1_ctrl_states.root_pos;
        a1_ctrl_states.foot_vel_world.block<3, 1>(0, i) = a1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + a1_ctrl_states.root_lin_vel;
    }
}

void IsaacA1ROS::imu_callback(const sensor_msgs::Imu::ConstPtr &imu) {
//    double wx = imu->angular_velocity.x;
    a1_ctrl_states.imu_acc = Eigen::Vector3d(
            acc_x.CalculateAverage(imu->linear_acceleration.x),
            acc_y.CalculateAverage(imu->linear_acceleration.y),
            acc_z.CalculateAverage(imu->linear_acceleration.z)
    );
    a1_ctrl_states.imu_ang_vel = Eigen::Vector3d(
            gyro_x.CalculateAverage(imu->angular_velocity.x),
            gyro_y.CalculateAverage(imu->angular_velocity.y),
            gyro_z.CalculateAverage(imu->angular_velocity.z)
    );


    return;
}

void IsaacA1ROS::joint_state_callback(const sensor_msgs::JointState::ConstPtr &a1_state) {
    // ROS_INFO("received");
    std::map<std::string, double> joint_positions;
    // FL
    joint_positions["FL_hip_joint"] = a1_state->position[0];
    joint_positions["FL_thigh_joint"] = a1_state->position[1];
    joint_positions["FL_calf_joint"] = a1_state->position[2];
    // FR
    joint_positions["FR_hip_joint"] = a1_state->position[3];
    joint_positions["FR_thigh_joint"] = a1_state->position[4];
    joint_positions["FR_calf_joint"] = a1_state->position[5];
    // RL
    joint_positions["RL_hip_joint"] = a1_state->position[6];
    joint_positions["RL_thigh_joint"] = a1_state->position[7];
    joint_positions["RL_calf_joint"] = a1_state->position[8];
    // RR
    joint_positions["RR_hip_joint"] = a1_state->position[9];
    joint_positions["RR_thigh_joint"] = a1_state->position[10];
    joint_positions["RR_calf_joint"] = a1_state->position[11];

    // update
    for (int i = 0; i < NUM_DOF; ++i) {
        a1_ctrl_states.joint_pos[i] = a1_state->position[i];
        a1_ctrl_states.joint_vel[i] = a1_state->velocity[i];
    }
    for (int i = 0; i < NUM_LEG; ++i) {
        a1_ctrl_states.foot_force[i] = a1_state->effort[NUM_DOF + i];
        a1_ctrl_states.isaac_contact_flag[i] = a1_state->position[NUM_DOF + i];
    }
}

void IsaacA1ROS::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // left updown
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
        joy_cmd_ctrl_state_change_request = true;
    }

    // right updown
    joy_cmd_velx = joy_msg->axes[5] * JOY_CMD_VELX_MAX;
    // right horiz
    joy_cmd_vely = joy_msg->axes[2] * JOY_CMD_VELY_MAX;
    // left horiz
    joy_cmd_yaw_rate = joy_msg->axes[0] * JOY_CMD_YAW_MAX;
    // up-down button
    joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
    // left-right button
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX;

    // lb
    if (joy_msg->buttons[4] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}