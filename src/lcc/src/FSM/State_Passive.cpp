/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Passive.h"

State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

void State_Passive::enter(){
    // if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].q = 0;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 8;
            _lowCmd->motorCmd[i].tau = 0;
        }
    // }
    // else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
    //     for(int i=0; i<12; i++){
    //         _lowCmd->motorCmd[i].mode = 10;
    //         _lowCmd->motorCmd[i].q = 0;
    //         _lowCmd->motorCmd[i].dq = 0;
    //         _lowCmd->motorCmd[i].Kp = 0;
    //         _lowCmd->motorCmd[i].Kd = 3;
    //         _lowCmd->motorCmd[i].tau = 0;
    //     }
    // }

    _ctrlComp->setAllSwing();
}

void State_Passive::run(){
    _torqueCtrl();
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}

void State_Passive::_torqueCtrl(){

    _Kd = Vec3( 100,  100, 100).asDiagonal();

    Vec34 vel34;
    Vec34 force34;
    Vec12 torque12;
    vel34.setZero();
    force34.setZero();
    torque12.setZero();

    vel34 = _ctrlComp->robotModel->getFeet2BVelocities(*_lowState,FrameType::HIP );

    for (int i = 0; i < 4; ++i){
        force34.block< 3, 1>( 0, i) = _Kd*(-vel34.block< 3, 1>( 0, i) );
    }

    Vec12 _q = vec34ToVec12(_lowState->getQ()); 
    torque12 = _ctrlComp->robotModel->getTau( _q, force34);

    _lowCmd->setTau(torque12);
    // torque12.setZero();
    // _lowCmd->setTau(torque12);
}