 
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSM/FSMState.h"

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:

    float _duration = 500;   //steps
    float _percent = 0;       //%

    //-------lcc------//
    #if IS_THIS_A_HEXAPOD
        // float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
        //                     0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
        float _startPos[18];
        Vec18 _targetPos2;
        Vec36 _initFeetPos, _feetPos, _feetPos2;
    #else
        float _targetPos[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
        float _startPos[12];
        Vec12 _targetPos2;
        Vec34 _initFeetPos, _feetPos, _feetPos2;
    #endif

    Mat3 _Kp, _Kd;
    void _torqueCtrl();
};

#endif  // FIXEDSTAND_H