//
// Created by zixin on 12/12/21.
//
#include <iostream>
#include <ctime>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include "../ConvexMpc.h"
#include "../A1CtrlStates.h"
#include "../A1Params.h"

int main(int, char **) {
    A1CtrlStates state;
    state.reset(); // 0, 1, 2, 3: FL, FR, RL, RR

    state.robot_mass = 15;
    state.a1_trunk_inertia << 0.0158533, 0.0, 0.0,
            0.0, 0.0377999, 0.0,
            0.0, 0.0, 0.0456542;

    state.root_euler << 0.0, 0.0, 0.0;

    Eigen::AngleAxisd rollAngle(state.root_euler[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(state.root_euler[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(state.root_euler[2], Eigen::Vector3d::UnitX());
    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    state.root_rot_mat = q.matrix();

    state.root_pos << 0.0, 0.0, 0.15;
    state.root_ang_vel << 0.0, 0.0, 0.0;
    state.root_lin_vel << 0.0, 0.0, 0.0;
    state.root_euler_d << 0.0, 0.0, 0.0;
    state.root_ang_vel_d << 0.0, 0.0, 0.0;
    state.root_lin_vel_d << 0.0, 0.0, 0.0;
    // 0, 1, 2, 3: FL, FR, RL, RR
    state.foot_pos_rel << 0.17, 0.17, -0.17, -0.17,
            0.15, -0.15, 0.15, -0.15,
            -0.35, -0.35, -0.35, -0.35;

    state.contacts[0] = true;
    state.contacts[1] = false;
    state.contacts[2] = true;
    state.contacts[3] = false;

    double dt = 0.0025;

    Eigen::VectorXd q_weights(13);
    Eigen::VectorXd r_weights(12);
    q_weights << 1.0, 1.0, 1.0,
            0.0, 0.0, 50.0,
            0.0, 0.0, 1.0,
            1.0, 1.0, 1.0,
            0.0;
    r_weights << 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6;
    ConvexMpc mpc_solver = ConvexMpc(q_weights, r_weights);
    mpc_solver.reset();

    // initialize the mpc state at the first time step
    state.mpc_states.resize(13);
    state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
            state.root_pos[0], state.root_pos[1], state.root_pos[2],
            state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
            state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
            -9.8;

    // initialize the desired mpc states trajectory
    state.root_lin_vel_d_world = state.root_rot_mat * state.root_lin_vel_d;
    state.mpc_states_d.resize(13 * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        state.mpc_states_d.segment(i * 13, 13)
                <<
                state.root_euler_d[0],
                state.root_euler_d[1],
                state.root_euler[2] + state.root_ang_vel_d[2] * dt * (i + 1),
                state.root_pos[0] + state.root_lin_vel_d_world[0] * dt * (i + 1),
                state.root_pos[1] + state.root_lin_vel_d_world[1] * dt * (i + 1),
                state.root_pos[2] + state.root_lin_vel_d_world[1] * dt * (i + 1),
                state.root_ang_vel_d[0],
                state.root_ang_vel_d[1],
                state.root_ang_vel_d[2],
                state.root_lin_vel_d_world[0],
                state.root_lin_vel_d_world[1],
                state.root_lin_vel_d_world[2],
                -9.8;
    }

    // a single A_c is computed for the entire reference trajectory using the average value of each euler angles during the reference trajectory
    Eigen::Vector3d avg_root_euler_in_horizon;
    avg_root_euler_in_horizon
            <<
            (state.root_euler[0] + state.root_euler[0] + state.root_ang_vel_d[0] * dt * PLAN_HORIZON) / (PLAN_HORIZON + 1),
            (state.root_euler[1] + state.root_euler[1] + state.root_ang_vel_d[1] * dt * PLAN_HORIZON) / (PLAN_HORIZON + 1),
            (state.root_euler[2] + state.root_euler[2] + state.root_ang_vel_d[2] * dt * PLAN_HORIZON) / (PLAN_HORIZON + 1);

    mpc_solver.calculate_A_mat_c(avg_root_euler_in_horizon);

    // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
    // from the reference trajectory and foot placement controller
    state.foot_pos_abs_mpc = state.foot_pos_rel;
    for (int i = 0; i < PLAN_HORIZON; i++) {
        // calculate current B_c matrix
        mpc_solver.calculate_B_mat_c(state.robot_mass,
                                     state.a1_trunk_inertia,
                                     state.root_rot_mat,
                                     state.foot_pos_abs_mpc);
        state.foot_pos_abs_mpc.block<3, 1>(0, 0) = state.foot_pos_abs_mpc.block<3, 1>(0, 0) - state.root_lin_vel_d * dt;
        state.foot_pos_abs_mpc.block<3, 1>(0, 1) = state.foot_pos_abs_mpc.block<3, 1>(0, 1) - state.root_lin_vel_d * dt;
        state.foot_pos_abs_mpc.block<3, 1>(0, 2) = state.foot_pos_abs_mpc.block<3, 1>(0, 2) - state.root_lin_vel_d * dt;
        state.foot_pos_abs_mpc.block<3, 1>(0, 3) = state.foot_pos_abs_mpc.block<3, 1>(0, 3) - state.root_lin_vel_d * dt;

        // state space discretization, calculate A_d and current B_d
        mpc_solver.state_space_discretization(dt);

        // store current B_d matrix
        mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
    }

    // calculate QP matrices
    mpc_solver.calculate_qp_mats(state);

    // solve
    std::clock_t start;
    start = std::clock();

    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(12 * PLAN_HORIZON);
    solver.data()->setNumberOfConstraints(20 * PLAN_HORIZON);
    solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
    solver.data()->setHessianMatrix(mpc_solver.hessian);
    solver.data()->setGradient(mpc_solver.gradient);
    solver.data()->setLowerBound(mpc_solver.lb);
    solver.data()->setUpperBound(mpc_solver.ub);

//        std::cout << "linear_constraints size: " << mpc_solver.linear_constraints.rows() << "X" << mpc_solver.linear_constraints.cols() << std::endl;
//        std::cout << "hessian size: " << mpc_solver.hessian.rows() << "X" << mpc_solver.hessian.cols() << std::endl;
//        std::cout << "gradient size: " << mpc_solver.gradient.size() << std::endl;
//        std::cout << "lb size: " << mpc_solver.lb.size() << std::endl;
//        std::cout << "lb size: " << mpc_solver.lb.size() << std::endl;

    solver.initSolver();
    solver.solve();

    Eigen::VectorXd solution = solver.getSolution();

    Eigen::Matrix<double, 3, 4> foot_forces_grf;
    for (int i = 0; i < 4; ++i) {
        foot_forces_grf.block<3, 1>(0, i) = solution.segment<3>(i * 3);
    }
    std::cout << foot_forces_grf << std::endl;

    std::cout << "Time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    return 0;
}
