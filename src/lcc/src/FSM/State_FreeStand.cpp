/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_FreeStand.h"

State_FreeStand::State_FreeStand(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::FREESTAND, "free stand"){
    _rowMax = 20 * M_PI / 180;
    _rowMin = -_rowMax;
    _pitchMax = 15 * M_PI / 180;
    _pitchMin = -_pitchMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;
    _heightMax = 0.04;
    _heightMin = -_heightMax;
}

void State_FreeStand::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }

    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
    }
    _initVecOX = _ctrlComp->robotModel->getX(*_lowState);
    _initVecXP = _ctrlComp->robotModel->getVecXP(*_lowState);

    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();
}

void State_FreeStand::run(){
    // _lowState->userValue = _lowState->userValue;

    _vecOP = _calcOP( invNormalize(_lowState->userValue.lx, _rowMin, _rowMax),
                     invNormalize(_lowState->userValue.ly, _pitchMin, _pitchMax),
                    -invNormalize(_lowState->userValue.rx, _yawMin, _yawMax),
                     invNormalize(_lowState->userValue.ry, _heightMin, _heightMax) );
    _calcCmd(_vecOP);
    _torqueCtrl();
}

void State_FreeStand::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_FreeStand::checkChange(){
    if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::TROTTING_5){
        return FSMStateName::TROTTING;
    }
    else{
        return FSMStateName::FREESTAND;
    }
}

Vec34 State_FreeStand::_calcOP(float row, float pitch, float yaw, float height){
    Vec3 vecXO = -_initVecOX;
    vecXO(2) += height;

    RotMat rotM = rpyToRotMat(row, pitch, yaw);

    HomoMat Tsb = homoMatrix(vecXO, rotM);
    HomoMat Tbs = homoMatrixInverse(Tsb);

    Vec4 tempVec4;
    Vec34 vecOP;
    for(int i(0); i<4; ++i){
        tempVec4 = Tbs * homoVec(_initVecXP.col(i));
        vecOP.col(i) = noHomoVec(tempVec4);
    }

    return vecOP;
}

void State_FreeStand::_calcCmd(Vec34 vecOP){
    Vec12 q = _ctrlComp->robotModel->getQ(vecOP, FrameType::BODY);
    _lowCmd->setQ(q);
}

void State_FreeStand::_torqueCtrl(){

    _Kp = Vec3(5000, 5000, 5000).asDiagonal();
    _Kd = Vec3( 200,  200, 200).asDiagonal();

    Vec34 pos34;
    Vec34 vel34;
    Vec34 _targetPos34;
    Vec34 force34;
    Vec12 torque12;
    pos34.setZero();
    vel34.setZero();
    force34.setZero();
    _targetPos34.setZero();
    torque12.setZero();

    pos34 = _ctrlComp->robotModel->getFeet2BPositions(*_lowState,FrameType::BODY );
    vel34 = _ctrlComp->robotModel->getFeet2BVelocities(*_lowState,FrameType::BODY );

    _targetPos34 = _vecOP;

    for (int i = 0; i < 4; ++i){
        force34.block< 3, 1>( 0, i) =  _Kp*(_targetPos34.block< 3, 1>( 0, i) - pos34.block< 3, 1>( 0, i) ) + _Kd*(-vel34.block< 3, 1>( 0, i) );
    }

    Vec12 _q = vec34ToVec12(_lowState->getQ()); 
    torque12 = _ctrlComp->robotModel->getTau( _q, force34);

    _lowCmd->setTau(torque12);


    // std::cout<<" _est->getPosition()\n"<< _est->getPosition()<<std::endl;
    // std::cout<<" _est->getFeetPos()\n"<< _est->getFeetPos()<<std::endl;
    // std::cout<<" _est->getFeetVel()\n"<< _est->getFeetVel()<<std::endl;
    // std::cout<<" _est->getPosFeet2BGlobal()\n"<< _est->getPosFeet2BGlobal()<<std::endl;
}