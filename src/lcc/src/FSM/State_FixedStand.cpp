/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include "FSM/State_FixedStand.h"

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){
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
        _startPos[i] = _lowState->motorState[i].q;
    }
    _ctrlComp->setAllStance();

    //-------lcc------//
    // _initFeetPos = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::HIP);
    _initFeetPos = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::BODY);
}

void State_FixedStand::run(){
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    for(int j=0; j<12; j++){
        _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
    }

    //-------lcc------//
    double h;
    // h = ( _ctrlComp->robotModel->_Legs[0]->_hipLinkLength + _ctrlComp->robotModel->_Legs[0]->_kneeLinkLength ) / 2 * 1.42;
    // h = ( _ctrlComp->robotModel->_Legs[0]->_hipLinkLength + _ctrlComp->robotModel->_Legs[0]->_kneeLinkLength ) / 2 * 1;
    // _feetPos <<
    // -0.006, -0.006, -0.006, -0.006,
    // -0.079, 0.079, -0.079, 0.079,
    // -h, -h, -h, -h;
    // _feetPos <<
    // -0.006, -0.006, -0.006, -0.006,
    // -0.079, 0.079, -0.079, 0.079,
    // -0.339, -0.339, -0.339, -0.339;

    _feetPos = _ctrlComp->robotModel->_feetPosNormalStand;//_feetPosNormalStand是body系下的
    for (int i = 0; i < 12; i++){
        _feetPos2(i) = (1 - _percent)*_initFeetPos(i) + _percent*_feetPos(i);  
    }
    
    // _targetPos2 = _ctrlComp->robotModel->getQ(_feetPos2, FrameType::HIP);// 基于hip关键做的足端位置规划
    _targetPos2 = _ctrlComp->robotModel->getQ(_feetPos2, FrameType::BODY);
    _lowCmd->setQ(_targetPos2);


    _torqueCtrl();


    // std::cout<<"  _feetPos \n"<< _feetPos <<std::endl;

    // _initFeetPos = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::HIP);
    // _feetPos = _initFeetPos;
    // _initPos = _initFeetPos.col(0);

    // std::cout<<"  getHip2B 0 \n"<< _ctrlComp->robotModel->_Legs[0]->getHip2B() <<std::endl;
    // std::cout<<"  getHip2B 1\n"<< _ctrlComp->robotModel->_Legs[1]->getHip2B() <<std::endl;
    // std::cout<<"  getHip2B 2 \n"<< _ctrlComp->robotModel->_Legs[2]->getHip2B() <<std::endl;
    // std::cout<<"  getHip2B 3\n"<< _ctrlComp->robotModel->_Legs[3]->getHip2B() <<std::endl;
    // std::cout<<"  _initFeetPos BODY \n"<< _ctrlComp->robotModel->getFeet2BPositions(*_lowState,FrameType::BODY ) <<std::endl;
    // std::cout<<"  _initFeetPos HIP\n"<< _ctrlComp->robotModel->getFeet2BPositions(*_lowState,FrameType::HIP ) <<std::endl;
    // std::cout<<"  _initPos \n"<< _initPos <<std::endl;
    // printf("\n---------lcc next----------\n");

    // _ctrlComp->setAllSwing();

}

void State_FixedStand::exit(){
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange(){
    if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::FREESTAND_3){
        return FSMStateName::FREESTAND;
    }
    else if(_lowState->userCmd == UserCommand::TROTTING_5){
        return FSMStateName::TROTTING;
    }
    else if(_lowState->userCmd == UserCommand::BALANCE_TEST0){
        return FSMStateName::BALANCETEST;
    }
    else if(_lowState->userCmd == UserCommand::SWING_TEST9){
        return FSMStateName::SWINGTEST;
    }
    else if(_lowState->userCmd == UserCommand::SETP_TEST8){
        return FSMStateName::STEPTEST;
    }
    else if(_lowState->userCmd == UserCommand::A1MPC_6){
        return FSMStateName::A1MPC;
    }
    else if(_lowState->userCmd == UserCommand::VMC_4){  //lcc 20240523
        return FSMStateName::VMC;
    }
#ifdef COMPILE_WITH_MOVE_BASE
    else if(_lowState->userCmd == UserCommand::L2_Y){
        return FSMStateName::MOVE_BASE;
    }
#endif  // COMPILE_WITH_MOVE_BASE
    else{
        return FSMStateName::FIXEDSTAND;
    }
}

void State_FixedStand::_torqueCtrl(){

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

    _targetPos34 = _feetPos2;

    for (int i = 0; i < 4; ++i){
        force34.block< 3, 1>( 0, i) =  _Kp*(_targetPos34.block< 3, 1>( 0, i) - pos34.block< 3, 1>( 0, i) ) + _Kd*(-vel34.block< 3, 1>( 0, i) );
    }

    Vec12 _q = vec34ToVec12(_lowState->getQ()); 
    torque12 = _ctrlComp->robotModel->getTau( _q, force34);

    // std::cout<<" pos34 \n"<< pos34 <<std::endl;
    // std::cout<<" _targetPos34 \n"<< _targetPos34 <<std::endl;
    // std::cout<<" vel34 \n"<< vel34 <<std::endl;
    // std::cout<<" force34 \n"<< force34 <<std::endl;
    // std::cout<<" _q \n"<< _q.transpose() <<std::endl;
    // std::cout<<" torque12 \n"<< torque12.transpose() <<std::endl;

    _lowCmd->setTau(torque12);
}