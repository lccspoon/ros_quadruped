 
#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

class State_Passive : public FSMState{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    void _torqueCtrl();
    Mat3 _Kp, _Kd;


    float _duration = 500;   //steps
    float _percent = 0;       //%
    // float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
    //                     0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
    float _startPos[18];
    Vec36 _targetPos2;
    Vec36 _initFeetPos, _feetPos;
};

#endif  // PASSIVE_H