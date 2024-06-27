 
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.position = new State_Position(_ctrlComp);
    _stateList.a1mpc = new State_A1MPC(_ctrlComp);//lcc 20240416
    _stateList.qp = new State_QP(_ctrlComp);//lcc 20240523
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

unsigned int _waitCount;
void FSM::run(){
    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->runWaveGen();
    // ++_waitCount;
    // if (_waitCount > 5000)
    // {
    //     _ctrlComp->estimator->run();
    // }
    _ctrlComp->estimator->run();

    // _ctrlComp->lowState->getQ_Hex()
    // std::cout << "getQ_Hex:\n" <<_ctrlComp->lowState->getQ_Hex()<< std::endl;

    // std::cout<<" _posBody: \n"<< _ctrlComp->estimator->getPosition() <<std::endl;
    // std::cout<<" _velBody: \n"<< _ctrlComp->estimator->getFeetVel() <<std::endl;

    // std::cout<<" _posFeet2BGlobal: \n"<< _ctrlComp->estimator->getPosFeet2BGlobal() <<std::endl;
    // std::cout<<" _posFeetGlobal: \n"<< _ctrlComp->estimator->getFeetPos() <<std::endl;
    // std::cout<<" _velFeetGlobal: \n"<< _ctrlComp->estimator->getFeetVel() <<std::endl;
    // printf("\n--------     next      --------\n");

    // std::cout<<" (*contact_hex): \n"<< (*_ctrlComp->contact_hex).transpose() <<std::endl;
    // std::cout<<" getFeetPos: \n"<< _ctrlComp->estimator->getFeetPos() <<std::endl;

    // _B2G_RotMat = _lowState->getRotMat();//机身 到 世界 的变化矩阵
    // _G2B_RotMat = _B2G_RotMat.transpose();//世界 到 机身 的变化矩阵

    // std::cout<<" getRotMat: \n"<< _ctrlComp->lowState->getRotMat()<<std::endl;
    // std::cout<<" getYaw: \n"<< _ctrlComp->lowState->getYaw()<<std::endl;
    // std::cout<<" getPosition: \n"<< _ctrlComp->estimator->getPosition()<<std::endl;

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
    case FSMStateName::POSITION:
        return _stateList.position;
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
    case FSMStateName::QP:    //lcc 20240523
        return _stateList.qp;
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