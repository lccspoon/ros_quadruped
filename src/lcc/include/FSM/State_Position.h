 
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "Gait/GaitGenerator_P.h"
#include "control/BalanceCtrl.h"
#include "control/TerrianEsti.h"
#include "Gait/SupportFeetEndP.h"
#include "Gait/SupportTrajectory.h"

class State_Position : public FSMState{
public:
    State_Position(CtrlComponents *ctrlComp);
    ~State_Position();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    // void setHighCmd(double vx, double vy, double wz);
private:
    void calcTau();
    void calcCmd();
    void calcP();
    virtual void getUserCmd();
    // void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    GaitGenerator_P  *_gait_P;
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

    //QP lcc 20240604
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

    SupportFeetEndP *_spf;
    SupportTrajectory *_spt;
    Vec36 support_leg_p;
    Vec36 _posSupportLeg_P, _velSupportLeg_P;
    Vec36 _posSwingLeg_P, _velSwingLeg_P;
    Vec36 _posFeet2BGoal_P;


    //lcc 20240624:机器人姿态与足端位置控制，参考书P77
    Vec3 _initVecOX;
    Vec36 _initVecXP;
    float _rowMax, _rowMin;
    float _pitchMax, _pitchMin;
    float _yawMax, _yawMin;
    float _heightMax, _heightMin;
    Vec36 _vecOP;
    Vec36 _calcOP(float row, float pitch, float yaw, float height);
    Vec3 adj_RPY_P, adj_RPY_P_past;
};

#endif  // TROTTING_H