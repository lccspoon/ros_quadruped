/*
    creat by lcc 20250414
*/

#ifndef CONTROL_WBC_H
#define CONTROL_WBC_H

#include "control/BalanceCtrl_WBC.h" //构造QP优化标准形
#include "control/NullSpaceOp_wbc.h" //零空间优化
#include "control/Pinocchio_WBC.h"   //获取机器人动力学参数

class Control_WBC
{
// private:
    /* data */
public:
    Control_WBC(/* args */);

    NullSpaceOp_wbc ns_wbc;
    BalanceCtrl_WBC balance_wbc;
    Pinocchio_WBC pino_wbc;

    Eigen::Matrix<double, 18, 1> wbc_run( 
    Eigen::Matrix<double, 25, 1> _q_with_quat, 
    Eigen::Matrix<double, 24, 1> _q_with_rpy, 
    Eigen::Matrix<double, 24, 1> _qd, 
    VecInt6 _contact_hex,

    Eigen::Matrix<double, 3, 1> x_2, 
    Eigen::Matrix<double, 3, 1> x_2_d,
    Eigen::Matrix<double, 3, 1> xd_2,
    Eigen::Matrix<double, 3, 1> xd_2_d,
    Eigen::Matrix<double, 3, 24> J_2,

    Eigen::Matrix<double, 3, 1> x_3, 
    Eigen::Matrix<double, 3, 1> x_3_d,
    Eigen::Matrix<double, 3, 1> xd_3,
    Eigen::Matrix<double, 3, 1> xd_3_d,
    Eigen::Matrix<double, 3, 24> J_3,

    Eigen::Matrix<double, 9, 1> x_4, 
    Eigen::Matrix<double, 9, 1> x_4_d,
    Eigen::Matrix<double, 9, 1> xd_4,
    Eigen::Matrix<double, 9, 1> xd_4_d,

    Eigen::Matrix<double, 3, 6> forec_mpc
    );

    Eigen::Matrix<double, 18, 1> ret_q_cmd(void);
    Eigen::Matrix<double, 18, 1> ret_qd_cmd(void);

};




#endif