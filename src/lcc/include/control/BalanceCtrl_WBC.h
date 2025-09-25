#ifndef BALANCECTRL_WBC_H
#define BALANCECTRL_WBC_H

#include "common/mathTypes.h"
#include "common/unitreeRobot.h"
#include <Eigen/Dense>

class BalanceCtrl_WBC {
public:
    BalanceCtrl_WBC();

    Eigen::Matrix<double, 9, 1> calF(
        const Vec24& qdd_cmd,
        const Eigen::Matrix<double, 9, 1>& f_mpc,
        const VecInt6& contact,
        Eigen::Matrix<double, 24, 24> _M,
        Eigen::Matrix<double, 24, 1> _C,
        Eigen::Matrix<double, 24, 9> _Jc_T
    );
    void runTest();

private:
    void calConstraints(const VecInt6& contact);
    void solveQP(const Vec24& qdd_cmd, const Eigen::Matrix<double, 9, 1>& f_mpc);

    double _mass;
    double _fricRatio;

    Eigen::Matrix<double, 24, 24> M;
    Eigen::Matrix<double, 24, 1> C;
    Eigen::Matrix<double, 24, 9> Jc_T;

    Eigen::Matrix<double, 6, 24> Sf;
    Eigen::Matrix<double, 6, 6> Q1;
    Eigen::Matrix<double, 9, 9> Q2;
    Eigen::Matrix<double, 5, 3> _fricMat;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> G, CE, CI;
    Eigen::Matrix<double, Eigen::Dynamic, 1> g0, ce0, ci0;

    Eigen::Matrix<double, 9, 1> fc;
    Vec6 delta_qdd;
    Eigen::Matrix<double, 9, 1> delta_f;
};

#endif