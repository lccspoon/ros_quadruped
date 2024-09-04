 
#include <iostream>
#include "FSM/State_FixedStand.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
std::vector<double> radians_to_degrees(const std::vector<double>& radians) {
    std::vector<double> degrees;
    for (double radian : radians) {
        double degree = radian * (180.0 / M_PI);
        degree = fmod(degree, 360.0);
        if (degree < 0) {
            degree += 360.0;
        }
        degrees.push_back(degree);
    }
    return degrees;
}

State_FixedStand::State_FixedStand(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::FIXEDSTAND, "fixed stand"){}

void State_FixedStand::enter(){

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
    
    USVLCC_SETZERO = true; 
    for(int i=0; i<NUM_DOF_W; i++){ //lcc 20240809
        _lowCmd->motorCmd[i].dq = 0;
        _lowCmd->motorCmd[i].Kp = 200;
        _lowCmd->motorCmd[i].Kd = 5;
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
// double radToAngle = 3.1415926/180;
double radToAngle = 180/3.1415926;
void State_FixedStand::run(){

    //-------lcc------//
    _feetPos = _ctrlComp->sixlegdogModel->_feetPosNormalStand;//_feetPosNormalStand是body系下的
    _percent += (float)1/_duration;
    _percent = _percent > 1 ? 1 : _percent;
    for (int i = 0; i < NUM_DOF_W; i++){
        _feetPos2(i) = (1 - _percent)*_initFeetPos(i) + _percent*_feetPos(i);  
    }
    
    _targetPos2 = _ctrlComp->sixlegdogModel->getQ(_feetPos2, FrameType::BODY);
    // _targetPos2 = _ctrlComp->sixlegdogModel->getQ(_feetPos, FrameType::BODY);
    _lowCmd->setQ(  _targetPos2   );

    // _lowCmd->setQ(   vec36ToVec18( _ctrlComp->lowState->getQ_Hex()  ) );

    // Vec36 rec_q, cmd_q;
    // for (int i = 0; i < 18; i++)
    // {
    //     rec_q(i) = _lowState->motorState[i].q; 
    //     cmd_q(i) = _lowCmd->motorCmd[i].q;
    // }
    
    // std::cout<<"rec_q: \n"<< rec_q * radToAngle <<std::endl;
    // std::cout<<"cmd_q: \n"<< cmd_q * radToAngle <<std::endl;
    // std::cout<<"_initFeetPos: \n"<< _initFeetPos <<std::endl;
    // std::cout<<"  sixlegdogModel  getQ: \n"<< vec18ToVec36(_ctrlComp->sixlegdogModel->getQ(_initFeetPos, FrameType::BODY) ) * radToAngle<<std::endl;
    // std::cout<<" _posFeet2BGlobal BODY: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY) <<std::endl;
    // std::cout<<" _posFeet2BGlobal HIP: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;
    // printf(" \n  ----------------- State_FixedStand -------------------- \n ");




    // _lowCmd->setQ(  init_pos   );

    // // std::cout<<" _feetPos2 BODY: \n"<< _feetPos2 <<std::endl;
    // std::cout<<" _posFeet2BGlobal BODY: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY) <<std::endl;
    // std::cout<<" _posFeet2BGlobal HIP: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;

    // Vec36 pp;
    // pp = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP);
    // std::cout<<" getQ: \n"<< vec18ToVec36( _ctrlComp->sixlegdogModel->getQ(pp, FrameType::HIP)) * 180 /3.1415 <<std::endl;
    // std::cout<<" getQ_Hex: \n"<< _lowState->getQ_Hex() * 180 /3.1415  <<std::endl;

    _torqueCtrl();
    // std::cout<<" _posFeet2BGlobal: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;
    // std::cout<<" _posFeet2BGlobal: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY) <<std::endl;
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
    else if(_lowState->userCmd == UserCommand::SQUAT_C){
        return FSMStateName::SQUAT;
    }
    else if(_lowState->userCmd == UserCommand::POSITION_5){
        return FSMStateName::POSITION;
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
    else if(_lowState->userCmd == UserCommand::QP_4){  //lcc 20240523
        return FSMStateName::QP;
    }
    else if(_lowState->userCmd == UserCommand::POSREFLEX_7){  //lcc 20240627
        return FSMStateName::POSREFLEX;
    }
    else if(_lowState->userCmd == UserCommand::FORCE_POS_8){  //lcc 20240827
        return FSMStateName::FORCE_POS;
    }
    else if(_lowState->userCmd == UserCommand::MPC_FORCE_POS_0){  //lcc 20240903
        return FSMStateName::MPC_FORCE_POS;
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

    #if IS_THIS_A_HEXAPOD
        #if USE_A_REAL_HEXAPOD == true
            // _Kp = Vec3(500, 500, 500).asDiagonal();
            // _Kd = Vec3( 15,  15, 15).asDiagonal() ;
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