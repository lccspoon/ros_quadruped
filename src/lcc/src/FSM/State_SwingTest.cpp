/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_SwingTest.h"

State_SwingTest::State_SwingTest(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::SWINGTEST, "swingTest"){
    _xMin = -0.15;
    _xMax =  0.10;
    _yMin = -0.15;
    _yMax =  0.15;
    _zMin = -0.05;
    _zMax =  0.20;
}

#define TEST_LEG_NUM 5 

void State_SwingTest::enter(){
    for(int i=0; i<NUM_LEG_W; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }
    _lowCmd->setSwingGain(TEST_LEG_NUM);

    _Kp = Vec3(20, 20, 50).asDiagonal();
    _Kd = Vec3( 5,  5, 20).asDiagonal();

    for(int i=0; i<NUM_DOF_W; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
    }

    #if IS_THIS_A_HEXAPOD
    _initFeetPos = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState, FrameType::HIP);
    #else
    _initFeetPos = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::HIP);
    #endif


    _feetPos = _initFeetPos;
    _initPos = _initFeetPos.col(TEST_LEG_NUM);

    // std::cout<<"  _initFeetPos \n"<< _initFeetPos <<std::endl;
    // std::cout<<"  _initPos \n"<< _initPos <<std::endl;
    // printf("\n---------lcc next----------\n");

    _ctrlComp->setAllSwing();
}

void State_SwingTest::run(){
    if(_lowState->userValue.ly > 0){
        _posGoal(0) = invNormalize(_lowState->userValue.ly, _initPos(0), _initPos(0)+_xMax, 0, 1);
    }else{
        _posGoal(0) = invNormalize(_lowState->userValue.ly, _initPos(0)+_xMin, _initPos(0), -1, 0);
    }
    
    if(_lowState->userValue.lx > 0){
        _posGoal(1) = invNormalize(_lowState->userValue.lx, _initPos(1, 0), _initPos(1)+_yMax, 0, 1);
    }else{
        _posGoal(1) = invNormalize(_lowState->userValue.lx, _initPos(1)+_yMin, _initPos(1), -1, 0);
    }

    if(_lowState->userValue.ry > 0){
        _posGoal(2) = invNormalize(_lowState->userValue.ry, _initPos(2), _initPos(2)+_zMax, 0, 1);
    }else{
        _posGoal(2) = invNormalize(_lowState->userValue.ry, _initPos(2)+_zMin, _initPos(2), -1, 0);
    }

    _positionCtrl();
    _torqueCtrl();
    // // printf("\n---------lcc next----------\n");
}

void State_SwingTest::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_SwingTest::checkChange(){
    if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::SWINGTEST;
    }
}

void State_SwingTest::_positionCtrl(){
    _feetPos.col(TEST_LEG_NUM) = _posGoal;
    // _targetPos = _ctrlComp->robotModel->getQ(_feetPos, FrameType::HIP);
    // _lowCmd->setQ(_targetPos);

    // printf(" lx:%f ly:%f ry:%f \n",_lowState->userValue.lx, _lowState->userValue.ly, _lowState->userValue.ry );
    // std::cout<<"  _feetPos.col(TEST_LEG_NUM) \n"<< _feetPos.col(TEST_LEG_NUM).transpose() <<std::endl;

    // std::cout<<"  _feetPos.col(TEST_LEG_NUM) \n"<< _ctrlComp->robotModel->getFootPosition( *_lowState, TEST_LEG_NUM, FrameType::BODY).transpose() <<std::endl;

}

void State_SwingTest::_torqueCtrl(){

    #if IS_THIS_A_HEXAPOD

        // _Kp = Vec3(5000, 5000, 5000).asDiagonal();
        // _Kd = Vec3( 200,  200, 200).asDiagonal();
        _Kp = Vec3(3500, 3500, 3500).asDiagonal();
        _Kd = Vec3( 120,  120, 120).asDiagonal();
        Vec36 pos36;
        Vec36 vel36;
        Vec36 _targetPos36;
        Vec36 force36;
        Vec18 torque18;
        vel36.setZero();
        pos36.setZero();
        force36.setZero();
        _targetPos36.setZero();
        torque18.setZero();

        pos36 = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP );
        vel36 = _ctrlComp->sixlegdogModel->getFeet2BVelocities(*_lowState,FrameType::HIP );

        _targetPos36 = _feetPos;

        for (int i = 0; i < 6; ++i){
            force36.block< 3, 1>( 0, i) =  _Kp*(_targetPos36.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
        }

        Vec18 _q = vec36ToVec18(_lowState->getQ_Hex()); 
        torque18 = _ctrlComp->sixlegdogModel->getTau( _q, force36);
        // std::cout<<" _feetPos2 :\n"<< _feetPos<<std::endl;
        // std::cout<<" _feetPos :\n"<< _feetPos<<std::endl;
        // std::cout<<" pos36 :\n"<< pos36<<std::endl;
        // std::cout<<" torque18 :\n"<< vec18ToVec36(torque18)<<std::endl;

        _lowCmd->setTau(torque18);

    #else
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

    pos34 = _ctrlComp->robotModel->getFeet2BPositions(*_lowState,FrameType::HIP );
    vel34 = _ctrlComp->robotModel->getFeet2BVelocities(*_lowState,FrameType::HIP );

    _targetPos34 = _feetPos;

    for (int i = 0; i < 4; ++i){
        force34.block< 3, 1>( 0, i) =  _Kp*(_targetPos34.block< 3, 1>( 0, i) - pos34.block< 3, 1>( 0, i) ) + _Kd*(-vel34.block< 3, 1>( 0, i) );
    }

    Vec12 _q = vec34ToVec12(_lowState->getQ()); 
    torque12 = _ctrlComp->robotModel->getTau( _q, force34);

    _lowCmd->setTau(torque12);
    #endif

}