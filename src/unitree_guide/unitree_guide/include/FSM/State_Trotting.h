/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"

class State_Trotting : public FSMState{
public:
    State_Trotting(CtrlComponents *ctrlComp);
    ~State_Trotting();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

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

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);
};

#endif  // TROTTING_H