
#include "FSM/FSM.h"
#include "interface/IOSDK.h"
#include <iostream>
#include "interface/IOROS.h"

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.fixedSquat = new State_FixedSquat(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.position = new State_Position(_ctrlComp);
    _stateList.a1mpc = new State_A1MPC(_ctrlComp);//lcc 20240416
    _stateList.qp = new State_QP(_ctrlComp);//lcc 20240523
    _stateList.posReflex = new State_PosReflex(_ctrlComp);//lcc 20240627
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

    #if USE_A_REAL_HEXAPOD == true
        if( _ctrlComp->lowState->userFunctionMode.motor_disenable_flag == true ){
            spi.exit_close_loop();
        }
        else{

        }
        usleep(2000);
    #endif

    // std::cout<<"getQ_Hex:  \n"<< _ctrlComp->lowState->getQ_Hex() * 180/3.1415926 <<std::endl;
    if( _waitCount <= 100 )
    {
        sub_joint_p_local_temp_origin(1) = -sub_joint_p_local_temp_origin(1);
        sub_joint_p_local_temp_origin(2) = -sub_joint_p_local_temp_origin(2);

        sub_joint_p_local_temp_origin(4) = -sub_joint_p_local_temp_origin(4);
        sub_joint_p_local_temp_origin(5) = -sub_joint_p_local_temp_origin(5);

        sub_joint_p_local_temp_origin(7) = -sub_joint_p_local_temp_origin(7);
        sub_joint_p_local_temp_origin(8) = -sub_joint_p_local_temp_origin(8);

        sub_joint_p_local_temp_origin(10) = -sub_joint_p_local_temp_origin(10);
        sub_joint_p_local_temp_origin(11) = -sub_joint_p_local_temp_origin(11);

        sub_joint_p_local_temp_origin(13) = -sub_joint_p_local_temp_origin(13);
        sub_joint_p_local_temp_origin(14) = -sub_joint_p_local_temp_origin(14);

        sub_joint_p_local_temp_origin(16) = -sub_joint_p_local_temp_origin(16);
        sub_joint_p_local_temp_origin(17) = -sub_joint_p_local_temp_origin(17);

        sub_joint_p_local_temp_origin(0) = -sub_joint_p_local_temp_origin(0);
        sub_joint_p_local_temp_origin(1) = -sub_joint_p_local_temp_origin(1);
        sub_joint_p_local_temp_origin(5) = -sub_joint_p_local_temp_origin(5);
        sub_joint_p_local_temp_origin(6) = -sub_joint_p_local_temp_origin(6);
        sub_joint_p_local_temp_origin(8) = -sub_joint_p_local_temp_origin(8);
        sub_joint_p_local_temp_origin(10) = -sub_joint_p_local_temp_origin(10);
        sub_joint_p_local_temp_origin(12) = -sub_joint_p_local_temp_origin(12);
        sub_joint_p_local_temp_origin(14) = -sub_joint_p_local_temp_origin(14);
        sub_joint_p_local_temp_origin(16) = -sub_joint_p_local_temp_origin(16);
        _ctrlComp->lowCmd->setQ( vec36ToVec18( sub_joint_p_local_temp_origin )  );
    }
    _waitCount++;

    _startTime = getSystemTime();
    _ctrlComp->sendRecv();
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->run();

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
    case FSMStateName::POSREFLEX:    //lcc 20240523
        return _stateList.posReflex;
        break;
    case FSMStateName::SQUAT:    //lcc 20240523
        return _stateList.fixedSquat;
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