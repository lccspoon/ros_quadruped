//
// Created by zixin on 12/09/21.
//

#ifndef A1_CPP_CONVEXMPC_H
#define A1_CPP_CONVEXMPC_H

#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include <vector>
#include <chrono>


#include <Eigen/Dense>

#include <iostream>

#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

// #include "A1CtrlStates.h"
// #include "A1Params.h"
// #include "utils/Utils.h"

// mpc
#define PLAN_HORIZON 10
#define MPC_STATE_DIM 13
#define MPC_CONSTRAINT_DIM 20

// robot constant
#define NUM_LEG 4
#define NUM_DOF_PER_LEG 3
#define DIM_GRF 12
#define NUM_DOF 12

class ConvexMpc {
public:
    ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_);

    void reset();

    void calculate_A_mat_c(Eigen::Vector3d root_euler);

    void calculate_B_mat_c(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia, Eigen::Matrix3d root_rot_mat,
                           Eigen::Matrix<double, 3, NUM_LEG> foot_pos);

    void state_space_discretization(double dt);

    void calculate_qp_mats( Eigen::Matrix<double, MPC_STATE_DIM, 1>  mpc_states, 
                            Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1>  mpc_states_d,
                            bool contacts[NUM_LEG]) ;

    Eigen::Matrix3d skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
    }

//private:
    // parameters initialized with class ConvexMpc
    double mu;
    double fz_min;
    double fz_max;

//    Eigen::VectorXd q_weights_mpc; // (state_dim * horizon) x 1
//    Eigen::VectorXd r_weights_mpc; // (action_dim * horizon) x 1
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc;

    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;
    Eigen::SparseMatrix<double> Q_sparse;
    Eigen::SparseMatrix<double> R_sparse;
//    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM * PLAN_HORIZON> Q;
//    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> R;

    // parameters initialized in the function reset()
//    Eigen::MatrixXd A_mat_c;
//    Eigen::MatrixXd B_mat_c;
//    Eigen::MatrixXd AB_mat_c;
//
//    Eigen::MatrixXd A_mat_d;
//    Eigen::MatrixXd B_mat_d;
//    Eigen::MatrixXd AB_mat_d;
//
//    Eigen::MatrixXd A_qp;
//    Eigen::MatrixXd B_qp;

    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_c_list;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_c;

    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_d_list;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_d;

    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> A_qp;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> B_qp;
    Eigen::SparseMatrix<double> A_qp_sparse;
    Eigen::SparseMatrix<double> B_qp_sparse;

    // standard QP formulation
    // minimize 1/2 * x' * P * x + q' * x
    // subject to lb <= Ac * x <= ub
    Eigen::SparseMatrix<double> hessian; // P
//    Eigen::VectorXd gradient; // q
    Eigen::SparseMatrix<double> linear_constraints; // Ac
//    Eigen::VectorXd lb;
//    Eigen::VectorXd ub;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> gradient; // q
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> ub;


};

#endif //A1_CPP_CONVEXMPC_H