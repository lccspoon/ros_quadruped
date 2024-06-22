/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.a1mpc = new State_A1MPC(_ctrlComp);//lcc 20240416
    _stateList.vmc = new State_VMC(_ctrlComp);//lcc 20240523
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

// Vec3 getPosition_offset;
// Vec3 getVelocity_offset;
// Vec34 getFeetVel_offset;
// Vec34 getFeetPos_offset;
// Vec34 getFeetPos_Z_offset;
// Vec34 getPosFeet2BGlobal_offset;
// Vec34 getPosFeet2BGlobal_Z_offset;

void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();
    
    if ( _currentState->_lowState->userFunctionMode.function_test == true ){
    //     getVelocity_offset = -_ctrlComp->estimator->getVelocity();
    //     getFeetVel_offset = -_ctrlComp->estimator->getFeetVel();

    //     getFeetPos_Z_offset = _ctrlComp->robotModel-> _feetPosNormalStand;
    //     getFeetPos_Z_offset(2) = 0; getFeetPos_Z_offset(5) = 0;
    //     getFeetPos_Z_offset(8) = 0;getFeetPos_Z_offset(11) = 0;

    //     getPosFeet2BGlobal_Z_offset = _ctrlComp->robotModel-> _feetPosNormalStand;  //这个需要  xRsb

    //     if ( _currentState->_stateName == FSMStateName::PASSIVE ){
    //         getPosition_offset = -_ctrlComp->estimator->getPosition();

    //         getFeetPos_offset = -_ctrlComp->estimator->getFeetPos() + getFeetPos_Z_offset;

    //         getPosFeet2BGlobal_Z_offset(2) = 0; getPosFeet2BGlobal_Z_offset(5) = 0;
    //         getPosFeet2BGlobal_Z_offset(8) = 0;getPosFeet2BGlobal_Z_offset(11) = 0;
    //         getPosFeet2BGlobal_offset = -_ctrlComp->estimator->getPosFeet2BGlobal() + getPosFeet2BGlobal_Z_offset;

    //     }
    //     else{
    //         getPosition_offset = -_ctrlComp->estimator->getPosition() - Vec3(0, 0, _ctrlComp->robotModel-> _feetPosNormalStand(2));

    //         getFeetPos_offset = -_ctrlComp->estimator->getFeetPos() + getFeetPos_Z_offset;

    //         getPosFeet2BGlobal_offset = -_ctrlComp->estimator->getPosFeet2BGlobal() + getPosFeet2BGlobal_Z_offset;
    //     }
    // }
    // else{
    //     // std::cout<<" _est->getPosition()\n"<< _ctrlComp->estimator->getPosition() + getPosition_offset <<std::endl;
    //     // std::cout<<" _est->getVelocity()\n"<< _ctrlComp->estimator->getVelocity() + getVelocity_offset <<std::endl;
    //     // std::cout<<" _est->getFeetVel()\n"<< _ctrlComp->estimator->getFeetVel() + getFeetVel_offset <<std::endl;
    //     // std::cout<<" _ctrlComp->estimator->getFeetPos()\n"<< _ctrlComp->estimator->getFeetPos() + getFeetPos_offset <<std::endl;
    //     // std::cout<<" _ctrlComp->estimator->getPosFeet2BGlobal()\n"<< _ctrlComp->estimator->getPosFeet2BGlobal() + getPosFeet2BGlobal_offset <<std::endl;
    }

    

    if(!checkSafty()){
        _ctrlComp->ioInter->setPassive();
    }

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }
    
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
    case FSMStateName::A1MPC:    //lcc 20240416
        return _stateList.a1mpc;
        break;
    case FSMStateName::VMC:    //lcc 20240523
        return _stateList.vmc;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}