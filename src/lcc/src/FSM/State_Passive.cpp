 
#include "FSM/State_Passive.h"
#include "interface/KeyBoard.h"
State_Passive::State_Passive(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::PASSIVE, "passive"){}

Vec36 init_q;
void State_Passive::enter(){
    // if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
    //     for(int i=0; i<NUM_DOF_W; i++){
    //         _lowCmd->motorCmd[i].mode = 10;
    //         _lowCmd->motorCmd[i].q = 0;
    //         _lowCmd->motorCmd[i].dq = 0;
    //         _lowCmd->motorCmd[i].Kp = 0;
    //         _lowCmd->motorCmd[i].Kd = 8;
    //         _lowCmd->motorCmd[i].tau = 0;
    //     }
    // }
    // else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
    //     for(int i=0; i<NUM_DOF_W; i++){
    //         _lowCmd->motorCmd[i].mode = 10;
    //         _lowCmd->motorCmd[i].q = 0;
    //         _lowCmd->motorCmd[i].dq = 0;
    //         _lowCmd->motorCmd[i].Kp = 0;
    //         _lowCmd->motorCmd[i].Kd = 3;
    //         _lowCmd->motorCmd[i].tau = 0;
    //     }
    // }

    #if USE_A_REAL_HEXAPOD == true
        for(int i=0; i<NUM_DOF_W; i++){ //lcc 20240809
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 0;
            _lowCmd->motorCmd[i].Kd = 5;
            _lowCmd->motorCmd[i].tau = 0;
        }
    #endif
    
    _ctrlComp->setAllSwing();
    init_q = _ctrlComp->lowState->getQ_Hex();
    // printf(" \n  -------------- enter passive-state ----------------- \n ");
}

//  _posFeet2BGlobal HIP: 
//  0.0755  0.0791 -0.0093  0.0037 -0.0756 -0.0760
// -0.1057  0.1091 -0.2868  0.2872 -0.1103  0.2865
// -0.2811 -0.2782 -0.0925 -0.0894 -0.2800 -0.0866

//  _posFeet2BGlobal HIP: 
//  0.0767  0.0791 -0.0045  0.0059 -0.0756 -0.0762
// -0.1046  0.1091 -0.1141  0.2872 -0.1102  0.2864
// -0.2812 -0.2782 -0.2790 -0.0895 -0.2801 -0.0867

//  _posFeet2BGlobal HIP: 
//  0.1127  0.1160  0.0435  0.0440 -0.0258 -0.0292
// -0.0639  0.0687 -0.1019  0.0989 -0.1312  0.1261
// -0.2809 -0.2782 -0.2804 -0.2807 -0.2800 -0.2804

//  _posFeet2BGlobal HIP: 
//  0.1135  0.1181  0.0417  0.0424 -0.0264 -0.0291
// -0.0856  0.0897 -0.0977  0.0997 -0.1299  0.1261
// -0.2751 -0.2712 -0.2822 -0.2806 -0.2806 -0.2804

void State_Passive::run(){
    // double radd;
    // radd = 3.1415926/180;
    // _targetPos2.setZero();

    // _targetPos2(0) = 25 * radd;
    // _targetPos2(3) = 25 * radd;
    // _targetPos2(6) = 25 * radd;
    // _targetPos2(9) = 25 * radd;
    // _targetPos2(12) = 25 * radd;
    // _targetPos2(15) = 25 * radd;

    // _targetPos2(1) = 25 * radd;
    // _targetPos2(4) = 25 * radd;
    // _targetPos2(7) = 25 * radd;
    // _targetPos2(10) = 25 * radd;
    // _targetPos2(13) = 25 * radd;
    // _targetPos2(16) = 25 * radd;

    // _targetPos2(2) = 90 * radd;
    // _targetPos2(5) = 90 * radd;
    // _targetPos2(8) = 90 * radd;
    // _targetPos2(11) = 90 * radd;
    // _targetPos2(14) = 90 * radd;
    // _targetPos2(17) = 90 * radd;

    // _percent += (float)1/_duration;
    // _percent = _percent > 1 ? 1 : _percent;
    // for (int i = 0; i < NUM_DOF_W; i++){
    //     _feetPos(i) = (1 - _percent)*init_q(i) + _percent*_targetPos2(i);  
    // }
    // _ctrlComp->lowCmd->setQ( vec36ToVec18( _feetPos )  );

    // std::cout<<" _posFeet2BGlobal HIP: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;
    // std::cout<<" _posFeet2BGlobal BODY: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY) <<std::endl;

    // Vec36 pp;
    // pp = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP);
    // std::cout<<" getQ: \n"<< vec18ToVec36( _ctrlComp->sixlegdogModel->getQ(pp, FrameType::HIP)) * 180 /3.1415 <<std::endl;

    // Vec18 qz;
    // qz.setZero();
    // _ctrlComp->lowCmd->setQ( qz  );

    // std::cout<<" State_Passive: getQ_Hex: \n"<< _lowState->getQ_Hex() * 180 /3.1415  <<std::endl;
    // std::cout<<" _posFeet2BGlobal HIP: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::HIP) <<std::endl;
    // std::cout<<" _posFeet2BGlobal BODY: \n"<< _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY) <<std::endl;

    // printf(" \n  -------------- next passive-state ----------------- \n ");
    #if USE_A_REAL_HEXAPOD == true
        _ctrlComp->lowCmd->setQ( vec36ToVec18( _ctrlComp->lowState->getQ_Hex() )  );
    #endif

    _torqueCtrl();
}

void State_Passive::exit(){

}

FSMStateName State_Passive::checkChange(){
    if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::SQUAT_C){
        return FSMStateName::SQUAT;
    }
    else{
        return FSMStateName::PASSIVE;
    }
}

void State_Passive::_torqueCtrl(){

    #if IS_THIS_A_HEXAPOD
        #if USE_A_REAL_HEXAPOD == true
            _Kd = Vec3( 15,  15, 15).asDiagonal() ;
        #else
            _Kd = Vec3( 100,  100, 100).asDiagonal() * 1 ;
        #endif

    Vec36 vel36;
    Vec36 force36;
    Vec18 torque18;
    vel36.setZero();
    force36.setZero();
    torque18.setZero();

    vel36 = _ctrlComp->sixlegdogModel->getFeet2BVelocities(*_lowState,FrameType::HIP );

    for (int i = 0; i < 6; ++i){
        force36.block< 3, 1>( 0, i) = _Kd*(-vel36.block< 3, 1>( 0, i) );
    }
    Vec18 _q = vec36ToVec18(_lowState->getQ_Hex()); 
    torque18 = _ctrlComp->sixlegdogModel->getTau( _q, force36);
    // std::cout<<" torque18 :\n"<< vec18ToVec36(torque18)<<std::endl;
    _lowCmd->setTau(torque18);

    #else
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
    #endif


}