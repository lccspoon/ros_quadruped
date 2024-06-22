/*
    @author lcc
    @date 20240523
*/
#ifndef _VMC_H_
#define _VMC_H_

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "common/filter.h"
#include "control/TerrianEsti.h"
#include<cstdlib> 

class State_VMC : public FSMState{
public:
    State_VMC(CtrlComponents *ctrlComp);
    ~State_VMC();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    // void setHighCmd(double vx, double vy, double wz);
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    // void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    HexapodRobot *_sixlegdogModel;//lcc 20240611
    BalanceCtrl *_balCtrl;
    TerrianEsti terr;//lcc 20240604

    // Rob State
    Vec3  _posBody, _velBody; // 由Estimator得到的：world下body的位置和速度
    double _yaw, _dYaw; // IMU得到：world下偏航角和偏航角速度
    Vec36 _posFeetGlobal, _velFeetGlobal; // 由Estimator得到
    Vec36 _posFeet2BGlobal; // 由Estimator得到的：world下foot_end相对body的向量
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec18 _q; // 各关节角度

    // Robot command
    Vec3 _pcd; // world系下，机身目标位置
    Vec3 _vCmdGlobal, _vCmdBody; // world系下，机身目标速度；body系下，机身目标速度
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal; //world下，目标转动向量
    Vec36 _posFeetGlobalGoal, _velFeetGlobalGoal;// 由GaitGenerator得到的：足端目标位置和速度
    Vec36 _posFeet2BGoal, _velFeet2BGoal;//世界系下，足端p、v在机身系下的向量
    RotMat _Rd;// 目标姿态的旋转矩阵
    Vec3 _ddPcd, _dWbd; // 目标线加速度，角加速度
    Vec36 _forceFeetGlobal, _forceFeetBody;
    Vec36 _qGoal, _qdGoal;
    Vec18 _tau;

    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;//摆动阻尼系数
    Vec2 _vxLim, _vyLim, _wyawLim;
    
    Vec4 *_phase;
    VecInt4 *_contact;

    Vec6 *_phase_hex;
    VecInt6 *_contact_hex;

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);

    //VMC lcc 20240604
    Mat3 _Kp, _Kd;
    void _torqueCtrl();
    Vec36 _initFeetPos;
    Vec18 torque18;
    int init_cout;

    //terrian estimator lcc 20240604
    double body_h;
    //terrian estimator
    Vec3 *_Apla;
    Vec3 root_euler_d;
    #if TERRIANESTI_FOURLEG
        Vec34 _posFeet2BGlobal_te;
        VecInt4 *_contact_te;
    #endif

};

#endif  // TROTTING_H