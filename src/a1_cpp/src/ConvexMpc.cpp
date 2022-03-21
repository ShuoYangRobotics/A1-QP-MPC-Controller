//
// Created by zixin on 12/09/21.
//

#include "ConvexMpc.h"

ConvexMpc::ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_) {
    mu = 0.3;
    fz_min = 0.0;
    fz_max = 0.0;

    // reserve size for sparse matrix
    Q_sparse = Eigen::SparseMatrix<double>(MPC_STATE_DIM * PLAN_HORIZON,MPC_STATE_DIM * PLAN_HORIZON);
    R_sparse = Eigen::SparseMatrix<double>(NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);

    q_weights_mpc.resize(MPC_STATE_DIM * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        q_weights_mpc.segment(i * MPC_STATE_DIM, MPC_STATE_DIM) = q_weights_;
    }
    Q.diagonal() = 2*q_weights_mpc;
    for (int i = 0; i < MPC_STATE_DIM*PLAN_HORIZON; ++i) {
        Q_sparse.insert(i,i) = 2*q_weights_mpc(i);
    }

//    Q.setZero();
//    R.setZero();
//
//    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> Q_small;
//    Q_small.setZero();
//    for (int i = 0; i < MPC_STATE_DIM; ++i) {
//        Q_small(i, i) = q_weights_[i];
//    }
//    for (int i = 0; i < PLAN_HORIZON; ++i) {
//        Q.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, MPC_STATE_DIM * i) = Q_small;
//    }

    r_weights_mpc.resize(NUM_DOF * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        r_weights_mpc.segment(i * NUM_DOF, NUM_DOF) = r_weights_;
    }
    R.diagonal() = 2*r_weights_mpc;
    for (int i = 0; i < NUM_DOF*PLAN_HORIZON; ++i) {
        R_sparse.insert(i,i) = 2*r_weights_mpc(i);
    }

    linear_constraints.resize(MPC_CONSTRAINT_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);    
    for (int i = 0; i < NUM_LEG * PLAN_HORIZON; ++i) {
        linear_constraints.insert(0 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints.insert(1 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints.insert(2 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints.insert(3 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints.insert(4 + 5 * i, 2 + 3 * i) = 1;

        linear_constraints.insert(0 + 5 * i, 2 + 3 * i) = mu;
        linear_constraints.insert(1 + 5 * i, 2 + 3 * i) = -mu;
        linear_constraints.insert(2 + 5 * i, 2 + 3 * i) = mu;
        linear_constraints.insert(3 + 5 * i, 2 + 3 * i) = -mu;
    }

//    Eigen::Matrix<double, NUM_DOF, NUM_DOF> R_small;
//    R_small.setZero();
//    for (int i = 0; i < NUM_DOF; ++i) {
//        R_small(i, i) = r_weights_[i];
//    }
//    for (int i = 0; i < PLAN_HORIZON; ++i) {
//        R.block<NUM_DOF, NUM_DOF>(NUM_DOF * i, NUM_DOF * i) = R_small;
//    }
}

void ConvexMpc::reset() {
//    // continuous time state space model
//    A_mat_c.resize(state_dim, state_dim);
//    B_mat_c.resize(state_dim, action_dim);
//    AB_mat_c.resize(state_dim + action_dim, state_dim + action_dim);
//
//    // discrete time state space model
//    A_mat_d.resize(state_dim, state_dim);
//    B_mat_d.resize(state_dim, action_dim);
//    AB_mat_d.resize(state_dim + action_dim, state_dim + action_dim);
//
//    // MPC state space model
//    A_qp.resize(state_dim * PLAN_HORIZON, state_dim);
//    B_qp.resize(state_dim * PLAN_HORIZON, action_dim * PLAN_HORIZON);
//
//    // QP formulation
//    hessian.resize(action_dim * PLAN_HORIZON, action_dim * PLAN_HORIZON); // sparse
//    gradient.resize(action_dim * PLAN_HORIZON);
//    linear_constraints.resize(constraints_dim * PLAN_HORIZON, action_dim * PLAN_HORIZON); // sparse
//    lb.resize(constraints_dim * PLAN_HORIZON);
//    ub.resize(constraints_dim * PLAN_HORIZON);

    A_mat_c.setZero();
    B_mat_c.setZero();
    B_mat_c_list.setZero();
    AB_mat_c.setZero();

    A_mat_d.setZero();
    B_mat_d.setZero();
    B_mat_d_list.setZero();

    AB_mat_d.setZero();
    A_qp.setZero();
    B_qp.setZero();
    gradient.setZero();
    lb.setZero();
    ub.setZero();

}

void ConvexMpc::calculate_A_mat_c(Eigen::Vector3d root_euler) {
//    std::cout << "yaw: " << root_euler[2] << std::endl;
    double cos_yaw = cos(root_euler[2]);
    double sin_yaw = sin(root_euler[2]);
//    double cos_pitch = cos(root_euler[1]);
//    double tan_pitch = tan(root_euler[1]);

    Eigen::Matrix3d ang_vel_to_rpy_rate;

//    ang_vel_to_rpy_rate << cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0,
//            -sin_yaw, cos_yaw, 0,
//            cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1;

    ang_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
            -sin_yaw, cos_yaw, 0,
            0, 0, 1;

    A_mat_c.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    A_mat_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat_c(11, NUM_DOF) = 1;
}

void ConvexMpc::calculate_B_mat_c(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia, Eigen::Matrix3d root_rot_mat,
                                  Eigen::Matrix<double, 3, NUM_LEG> foot_pos) {
    // we need to calculate PLAN_HORIZON B matrices
    Eigen::Matrix3d a1_trunk_inertia_world;
    a1_trunk_inertia_world = root_rot_mat * a1_trunk_inertia * root_rot_mat.transpose();
    for (int i = 0; i < NUM_LEG; ++i) {
        B_mat_c.block<3, 3>(6, 3 * i) =
                a1_trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3, 1>(0, i));
        B_mat_c.block<3, 3>(9, 3 * i) =
                (1 / robot_mass) * Eigen::Matrix3d::Identity();
    }
}

void ConvexMpc::state_space_discretization(double dt) {
    // simplified exp 
    // TODO: this function is not necessary because A is actually sparse
    auto t1 = std::chrono::high_resolution_clock::now();
    // AB_mat_d = (dt * AB_mat_c).exp();
    A_mat_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A_mat_c*dt;
    B_mat_d = B_mat_c*dt;
    auto t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
//    std::cout << "IN DISCRETIZATION: matrix exp: " << ms_double_1.count() << "ms" << std::endl;
}

void ConvexMpc::calculate_qp_mats(A1CtrlStates &state) {
    // standard QP formulation
    // minimize 1/2 * x' * P * x + q' * x
    // subject to lb <= Ac * x <= ub
    // P: hessian
    // q: gradient
    // Ac: linear constraints

    // A_qp = [A,
    //         A^2,
    //         A^3,
    //         ...
    //         A^k]'

    // B_qp = [A^0*B(0),
    //         A^1*B(0),     B(1),
    //         A^2*B(0),     A*B(1),       B(2),
    //         ...
    //         A^(k-1)*B(0), A^(k-2)*B(1), A^(k-3)*B(2), ... B(k-1)]

    // auto t1 = std::chrono::high_resolution_clock::now();
    // calculate A_qp and B_qp
    // TODO: THIS PART TAKES AROUND 0.2MS!
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> tmp_mtx;

    // keep A_qp as a storage list 
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        if (i == 0) {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = A_mat_d;
        }
        else {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = 
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-1), 0)*A_mat_d;
        }
        for (int j = 0; j < i + 1; ++j) {
            if (i-j == 0) {
                B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                    B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            } else {
                B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                        A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-j-1), 0) 
                        * B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            }
        }
    }

    // auto t2 = std::chrono::high_resolution_clock::now();
    // calculate hessian
    // TODO: THIS PART TAKES AROUND 0.4MS!
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> dense_hessian;
//    dense_hessian = (B_qp.transpose() * Q * B_qp + R);
    dense_hessian = (B_qp.transpose() * Q * B_qp);
    dense_hessian += R;
    hessian = dense_hessian.sparseView();

    // auto t3 = std::chrono::high_resolution_clock::now();
    // calculate gradient
    Eigen::Matrix<double, 13*PLAN_HORIZON, 1> tmp_vec = A_qp* state.mpc_states;
    tmp_vec -= state.mpc_states_d;
    gradient = B_qp.transpose() * Q * tmp_vec;

    // auto t4 = std::chrono::high_resolution_clock::now();

    // auto t5 = std::chrono::high_resolution_clock::now();
    // calculate lower bound and upper bound
    fz_min = 0;
    fz_max = 180;

    Eigen::VectorXd lb_one_horizon(MPC_CONSTRAINT_DIM);
    Eigen::VectorXd ub_one_horizon(MPC_CONSTRAINT_DIM);
    for (int i = 0; i < NUM_LEG; ++i) {
        lb_one_horizon.segment<5>(i * 5) << 0,
                -OsqpEigen::INFTY,
                0,
                -OsqpEigen::INFTY,
                fz_min * state.contacts[i];
        ub_one_horizon.segment<5>(i * 5) << OsqpEigen::INFTY,
                0,
                OsqpEigen::INFTY,
                0,
                fz_max * state.contacts[i];
    }
    // std:: cout << lb_one_horizon.transpose() << std::endl;
    // std:: cout << ub_one_horizon.transpose() << std::endl;
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        lb.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = lb_one_horizon;
        ub.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = ub_one_horizon;
    }

    // auto t6 = std::chrono::high_resolution_clock::now();

    // std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
    // std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
    // std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
    // std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
    // std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;

    // std::cout << "IN CAL_QP_MATS: cal A_qp and B_qp: " << ms_double_1.count() << "ms" << std::endl;
    // std::cout << "IN CAL_QP_MATS: cal hessian: " << ms_double_2.count() << "ms" << std::endl;
    // std::cout << "IN CAL_QP_MATS: cal gradient: " << ms_double_3.count() << "ms" << std::endl;
    // std::cout << "IN CAL_QP_MATS: cal linear constraints: " << ms_double_4.count() << "ms" << std::endl;
    // std::cout << "IN CAL_QP_MATS: cal lb and ub: " << ms_double_5.count() << "ms" << std::endl;
}
