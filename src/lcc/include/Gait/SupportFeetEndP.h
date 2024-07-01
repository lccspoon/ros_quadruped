 
#ifndef SUPPORTFEETENDP_H_
#define SUPPORTFEETENDP_H_

#include "control/CtrlComponents.h"
#include "message/LowlevelState.h"

class SupportFeetEndP{
public:
    SupportFeetEndP(CtrlComponents *ctrlComp);
    ~SupportFeetEndP();
    // Vec3 calc(int legID, Vec2 vxyGoalGlobal, float dYawGoal, Vec3 _startP);
    Vec3 calc_support_fe(int legID, Vec2 vxyGoalGlobal, float dYawGoal, Vec3 P_Increment, Vec1_6 *terian_FootHold);
    Vec3 calc_swing_fe(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase, Vec3 P_Increment, Vec1_6 *terian_FootHold);
private:
    LowlevelState *_lowState;
    Estimator *_est;
    // QuadrupedRobot *_robModel;
    HexapodRobot *_robModel;

    Vec3 _nextStep, _footPos;
    Vec3 _bodyVelGlobal;        // linear velocity
    Vec3 _bodyAccGlobal;        // linear accelerator
    Vec3 _bodyWGlobal;          // angular velocity

    VecInt6 *_contact;
    Vec6 *_phase;
    // VecInt6 _contact_last;

    Vec6 _feetRadius, _feetInitAngle;
    float _yaw, _dYaw, _nextYaw;

    float _Tstance, _Tswing;
    float _kx, _ky, _kyaw;

    Vec3 *_Apla; //lcc 

    Vec36 feetPosBody;
};

#endif  // FEETENDCAL_H
