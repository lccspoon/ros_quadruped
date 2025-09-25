 
#include <iostream>
#include "FSM/State_FixedSquat.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
// std::vector<double> radians_to_degrees(const std::vector<double>& radians) {
//     std::vector<double> degrees;
//     for (double radian : radians) {
//         double degree = radian * (180.0 / M_PI);
//         degree = fmod(degree, 360.0);
//         if (degree < 0) {
//             degree += 360.0;
//         }
//         degrees.push_back(degree);
//     }
//     return degrees;
// }

State_FixedSquat::State_FixedSquat(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::SQUAT, "fixed squat"){}

void State_FixedSquat::enter(){
    // for(int i=0; i<NUM_LEG_W; i++){
    //     if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
    //         _lowCmd->setSimStanceGain(i);
    //     }
    //     else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
    //         _lowCmd->setRealStanceGain(i);
    //     }
    //     _lowCmd->setZeroDq(i);
    //     _lowCmd->setZeroTau(i);
    // }

    for(int i=0; i<NUM_DOF_W; i++){ //lcc 20240809
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].Kp = 200;
        _lowCmd->motorCmd[i].Kd = 2;
        _lowCmd->motorCmd[i].tau = 0;
    }

    for(int i=0; i<NUM_DOF_W; i++){
        #if USE_A_REAL_HEXAPOD == true
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        #endif
        _startPos[i] = _lowState->motorState[i].q;
    }

    _ctrlComp->setAllStance();

    //-------lcc------//
    // _initFeetPos = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState, FrameType::HIP);
    // _initFeetPos = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState, FrameType::BODY);
    _initFeetPos = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState, FrameType::BODY);
}

void State_FixedSquat::run(){

    //-------lcc------//
    _feetPos = _ctrlComp->sixlegdogModel->_feetPosNormalSquat;//_feetPosNormalSquat是body系下的
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    for (int i = 0; i < NUM_DOF_W; i++){
        _feetPos2(i) = (1 - _percent)*_initFeetPos(i) + _percent*_feetPos(i);  
    }
    
    _targetPos2 = _ctrlComp->sixlegdogModel->getQ(_feetPos2, FrameType::BODY);
    // _targetPos2 = _ctrlComp->sixlegdogModel->getQ(_feetPos, FrameType::BODY);
    _lowCmd->setQ(  _targetPos2   );

    // _lowCmd->setQ(  init_pos   );

    // // std::cout<<" _feetPos2 BODY: \n"<< _feetPos2 <<std::endl;
    // std::cout<<" _posFeet2BGlobal BODY: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY) <<std::endl;
    // std::cout<<" _posFeet2BGlobal HIP: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;

    // Vec36 pp;
    // pp = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP);
    // std::cout<<" getQ: \n"<< vec18ToVec36( _ctrlComp->sixlegdogModel->getQ(pp, FrameType::HIP)) * 180 /3.1415 <<std::endl;
    // std::cout<<" getQ_Hex: \n"<< _lowState->getQ_Hex() * 180 /3.1415  <<std::endl;

    // Vec36 pp;
    // pp = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP);
    // std::cout<<" 逆运动学解算 getQ: \n"<< vec18ToVec36( _ctrlComp->sixlegdogModel->getQ(pp, FrameType::HIP)) * 180 /3.1415 <<std::endl;
    // std::cout<<" State_Passive: getQ_Hex: \n"<< _lowState->getQ_Hex() * 180 /3.1415  <<std::endl;
    // std::cout<<" 正运动学解算_posFeet2BGlobal HIP: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;

    // std::cout<<" 目标位置_feetPos: \n"<< _feetPos <<std::endl;
    // std::cout<<" 期望位置_feetPos2: \n"<< _feetPos2 <<std::endl;
    // std::cout<<" 发送角度_targetPos2: \n"<< _targetPos2 * 180 /3.1415 <<std::endl;


    _torqueCtrl();
    // std::cout<<" _posFeet2BGlobal: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;
    // std::cout<<" _posFeet2BGlobal: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY) <<std::endl;
}

void State_FixedSquat::exit(){
    _percent = 0;
}

FSMStateName State_FixedSquat::checkChange(){
    if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::SQUAT;
    }
}

void State_FixedSquat::_torqueCtrl(){

    #if IS_THIS_A_HEXAPOD
        #if USE_A_REAL_HEXAPOD == true
            // _Kp = Vec3(500, 500, 500).asDiagonal();
            // _Kd = Vec3( 15,  15, 15).asDiagonal() ;
            // _Kp = Vec3(400, 400, 400).asDiagonal();
            // _Kd = Vec3( 10,  10, 10).asDiagonal() ;
            _Kp = Vec3(100, 100, 100).asDiagonal();
            _Kd = Vec3( 1,  1, 1).asDiagonal() ;
            // _Kp = Vec3(35, 35, 35).asDiagonal();
            // _Kd = Vec3( 1,  1, 1).asDiagonal() ;
        #else
            // _Kp = Vec3(5000, 5000, 5000).asDiagonal();
            // _Kd = Vec3( 200,  200, 200).asDiagonal();
            _Kp = Vec3(3500, 3500, 3500).asDiagonal();
            _Kd = Vec3( 120,  120, 120).asDiagonal();
        #endif

        Vec36 pos36;
        Vec36 vel36;
        Vec36 force36;
        Vec18 torque18;
        vel36.setZero();
        force36.setZero();
        torque18.setZero();

        pos36 = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY );
        vel36 = _ctrlComp->sixlegdogModel->getFeet2BVelocities(*_lowState,FrameType::BODY );

        for (int i = 0; i < 6; ++i){
            force36.block< 3, 1>( 0, i) =  _Kp*(_feetPos2.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
            // force36.block< 3, 1>( 0, i) =  _Kp*(_feetPos.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
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
    #endif

}