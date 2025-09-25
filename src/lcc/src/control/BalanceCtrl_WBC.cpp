#include "control/BalanceCtrl_WBC.h"
#include "common/mathTools.h"
#include "common/timeMarker.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include <iostream>
#include <Eigen/SVD>

quadprogpp::Matrix<double> G_WBC, CE_WBC, CI_WBC;
quadprogpp::Vector<double> g0_WBC, ce0_WBC, ci0_WBC, x_WBC;

BalanceCtrl_WBC::BalanceCtrl_WBC() {
    _mass = 10;
    _fricRatio = 0.4;

    Sf.setZero();
    Sf.block<6, 6>(0, 0).setIdentity();

    Q1.setIdentity();
    Q1 *= 0.1;

    Q2.setIdentity();
    Q2 *= 100.0;

    _fricMat.resize(5, 3);
    _fricMat << 1.0, 0.0, _fricRatio,
               -1.0, 0.0, _fricRatio,
                0.0, 1.0, _fricRatio,
                0.0, -1.0, _fricRatio,
                0.0, 0.0, 1.0;

    fc.setZero();
    delta_qdd.setZero();
    delta_f.setZero();
}

Eigen::Matrix<double, 9, 1> BalanceCtrl_WBC::calF(
    const Vec24& qdd_cmd,
    const Eigen::Matrix<double, 9, 1>& f_mpc,
    const VecInt6& contact,
    Eigen::Matrix<double, 24, 24> _M,
    Eigen::Matrix<double, 24, 1> _C,
    Eigen::Matrix<double, 24, 9> _Jc_T
) {
    // if (_M.hasNaN() || _C.hasNaN() || _Jc_T.hasNaN() || qdd_cmd.hasNaN() || f_mpc.hasNaN()) {
    //     throw std::runtime_error("Input matrices contain NaN");
    // }
    // if (_M.rows() != 24 || _M.cols() != 24 || _C.rows() != 24 || _C.cols() != 1 || _Jc_T.rows() != 24 || _Jc_T.cols() != 9) {
    //     throw std::runtime_error("Invalid input matrix sizes");
    // }
    M = _M;
    C = _C;
    Jc_T = _Jc_T;

    // 重置 Jc_T，确保 z 力映射
    Jc_T.setZero();
    Jc_T.block<3,3>(0,0).setIdentity();
    Jc_T.block<3,3>(3,3).setIdentity();
    Jc_T.block<3,3>(6,6).setIdentity();
    Jc_T(2,5) = 1.0;
    Jc_T(2,8) = 1.0;

    calConstraints(contact);
    solveQP(qdd_cmd, f_mpc);
    return fc;
}

void BalanceCtrl_WBC::calConstraints(const VecInt6& contact) {
    int contactLegNum = 0;
    for (int i = 0; i < 6; ++i) {
        if (contact(i) == 1) {
            contactLegNum += 1;
        }
    }
    if (contactLegNum == 0) {
        throw std::runtime_error("No contact legs detected");
    }

    CI.resize(7 * contactLegNum, 9);
    ci0.resize(7 * contactLegNum);
    CI.setZero();
    ci0.setZero();

    int ciID = 0;
    for (int i = 0; i < 6; ++i) {
        if (contact(i) == 1) {
            int idx;
            if (contact(0) == 1 && contact(1) == 0) {
                if (i == 0) idx = 0;
                else if (i == 3) idx = 1;
                else if (i == 4) idx = 2;
                else throw std::runtime_error("Invalid contact leg index: " + std::to_string(i));
            } else if (contact(0) == 0 && contact(1) == 1) {
                if (i == 1) idx = 0;
                else if (i == 2) idx = 1;
                else if (i == 5) idx = 2;
                else throw std::runtime_error("Invalid contact leg index: " + std::to_string(i));
            } else {
                throw std::runtime_error("Unsupported contact pattern");
            }
            if (7 * ciID + 7 > CI.rows() || 3 * idx + 3 > CI.cols()) {
                throw std::runtime_error("Block operation out of bounds: ciID=" + std::to_string(ciID) + ", idx=" + std::to_string(idx));
            }
            CI.block(7 * ciID, 3 * idx, 5, 3) = _fricMat;
            CI(7 * ciID + 5, 3 * idx + 2) = 1.0;
            ci0(7 * ciID + 5) = 0.0;
            CI(7 * ciID + 6, 3 * idx + 2) = -1.0;
            ci0(7 * ciID + 6) = -1000.0; // 放宽约束
            ciID++;
        }
    }
}

void BalanceCtrl_WBC::solveQP(const Vec24& qdd_cmd, const Eigen::Matrix<double, 9, 1>& f_mpc) {
    int n_vars = 6 + 9;

    G.resize(n_vars, n_vars);
    g0.resize(n_vars);
    G.setZero();
    g0.setZero();

    if (Q1.rows() != 6 || Q2.rows() != 9) {
        throw std::runtime_error("Invalid Q1 or Q2 sizes");
    }
    G.block<6, 6>(0, 0) = 2.0 * Q1;
    G.block<9, 9>(6, 6) = 2.0 * Q2;

    int n_eq = 6;
    CE.resize(n_eq, n_vars);
    ce0.resize(n_eq);
    CE.setZero();
    ce0.setZero();

    Eigen::Matrix<double, 24, 6> Dqdd;
    Dqdd.setZero();
    Dqdd.block<6, 6>(0, 0).setIdentity();

    CE.block<6, 6>(0, 0) = Sf * M * Dqdd;
    CE.block<6, 9>(0, 6) = -Sf * Jc_T;

    Eigen::VectorXd Jc_f = Jc_T * f_mpc;
    Eigen::VectorXd M_qdd = M * qdd_cmd;
    ce0.segment<6>(0) = Sf * (Jc_f - M_qdd - C);

    int contactLegNum = CI.rows() / 7;
    if (contactLegNum == 0) {
        throw std::runtime_error("No valid constraints in CI");
    }

    int n = n_vars;
    int m = n_eq;
    int p = ci0.size();

    G_WBC.resize(n, n);
    CE_WBC.resize(n, m);
    CI_WBC.resize(n, p);
    g0_WBC.resize(n);
    ce0_WBC.resize(m);
    ci0_WBC.resize(p);
    x_WBC.resize(n);
    for (int i = 0; i < n; ++i) {
        x_WBC[i] = 0.0; // 初始化
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G_WBC[i][j] = G(i, j);
        }
        g0_WBC[i] = g0(i);
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE_WBC[i][j] = i < 6 ? CE(j, i) : 0.0;
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI_WBC[i][j] = i < 6 ? 0.0 : CI(j, i - 6);
        }
    }
    for (int i = 0; i < m; ++i) {
        ce0_WBC[i] = ce0(i);
    }
    for (int i = 0; i < p; ++i) {
        ci0_WBC[i] = ci0(i);
    }

    Eigen::MatrixXd G_eigen(n, n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G_eigen(i, j) = G_WBC[i][j];
        }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(G_eigen);
    if (es.eigenvalues().minCoeff() < -1e-6) {
        throw std::runtime_error("G matrix is not positive definite");
    }

    double value = solve_quadprog(G_WBC, g0_WBC, CE_WBC, ce0_WBC, CI_WBC, ci0_WBC, x_WBC);
    if (std::isinf(value) || std::isnan(value)) {
        fc = f_mpc; // QP 失败，回退
        return;
    }

    for (int i = 0; i < 6; ++i) {
        delta_qdd(i) = x_WBC[i];
    }
    for (int i = 0; i < 9; ++i) {
        delta_f(i) = x_WBC[i + 6];
    }
    fc = f_mpc + delta_f;
}

void BalanceCtrl_WBC::runTest() {
    Vec24 qdd_cmd = Vec24::Zero();
    Eigen::Matrix<double, 9,1> f_mpc = Eigen::Matrix<double, 9,1>::Zero();
    double mass = 10.0;
    double g = 9.81;
    double f_z = mass * g / 3.0;
    f_mpc(2) = f_z;
    f_mpc(5) = f_z;
    f_mpc(8) = f_z;
    VecInt6 contact;
    contact << 1, 0, 0, 1, 1, 0;
    Eigen::Matrix<double, 24, 24> M = Eigen::MatrixXd::Identity(24, 24) * 10.0;
    Eigen::Matrix<double, 24, 1> C = Eigen::VectorXd::Zero(24);
    C(2) = -mass * g;
    Eigen::Matrix<double, 24, 9> Jc_T = Eigen::MatrixXd::Zero(24, 9);
    Jc_T.block<3,3>(0,0).setIdentity();
    Jc_T.block<3,3>(3,3).setIdentity();
    Jc_T.block<3,3>(6,6).setIdentity();

    Eigen::Matrix<double, 9,1>  f;

    try {
        f = calF(qdd_cmd, f_mpc, contact, M, C, Jc_T);
    } catch (const std::exception&) {
    }

    std::cout << "aaTest f: " << f.transpose() << std::endl;
}