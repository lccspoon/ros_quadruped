 
#ifndef FREESTAND_H
#define FREESTAND_H

#include "FSM/FSMState.h"

class State_FreeStand : public FSMState{
public:
    State_FreeStand(CtrlComponents *ctrlComp);
    ~State_FreeStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
private:
    Vec3 _initVecOX;
    Vec36 _initVecXP;
    float _rowMax, _rowMin;
    float _pitchMax, _pitchMin;
    float _yawMax, _yawMin;
    float _heightMax, _heightMin;
    Vec36 _vecOP;

    Vec36 _calcOP(float row, float pitch, float yaw, float height);
    void _calcCmd(Vec36 vecOP);

    void _torqueCtrl();
    Mat3 _Kp, _Kd;
    
};

#endif  // FREESTAND_H