/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef A1MPC_H_
#define A1MPC_H_

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "control/ConvexMpc.h"
#include "control/TerrianEsti.h"
#include "common/filter.h"
#include<cstdlib> 

class State_A1MPC : public FSMState{
public:
    State_A1MPC(CtrlComponents *ctrlComp);
    ~State_A1MPC();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
private:
    void calcGrf();
    void calcQPf();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    bool checkStepOrNot();


    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;
    ConvexMpc *_convMpc;
    TerrianEsti terr;

    // Rob State
    Vec3  _posBody, _velBody; // 由Estimator得到的：world下body的位置和速度
    double _yaw, _dYaw; // IMU得到：world下偏航角和偏航角速度
    Vec34 _posFeetGlobal, _velFeetGlobal; // 由Estimator得到
    Vec34 _posFeet2BGlobal; // 由Estimator得到的：world下foot_end相对body的向量
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q; // 各关节角度

    // Robot command
    Vec3 _pcd; // world系下，机身目标位置
    Vec3 _vCmdGlobal, _vCmdBody; // world系下，机身目标速度；body系下，机身目标速度
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal; //world下，目标转动向量
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;// 由GaitGenerator得到的：足端目标位置和速度
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;// 目标姿态的旋转矩阵
    Vec3 _ddPcd, _dWbd; // 目标线加速度，角加速度
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;//摆动阻尼系数
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;

    // MPC parameters
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;
    Vec3 root_euler_d;

    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;
    OsqpEigen::Solver solver;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    int mpc_init_counter;

    //terrian estimator
    Vec3 *_Apla;

    Vec34 _posFeetGlobalGoal_last;
    Vec4 touch_flag;
};




#endif  // TROTTING_H