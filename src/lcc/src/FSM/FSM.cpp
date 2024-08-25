
#include "FSM/FSM.h"
#include "interface/IOSDK.h"
#include <iostream>
#include "interface/IOROS.h"
#include "interface/imu.h"
#include <thread>
FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.fixedSquat = new State_FixedSquat(_ctrlComp);//lcc 20240808
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
    // initialize();
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
Vec36 init_motor_set_q;
Vec36 dou_dong_angle;
bool get_motor_response_flag = false;
void FSM::run(){
    #if USE_A_REAL_HEXAPOD == true
        if( MOTOR_DISABEL_FLAG == true ){ // 'o'
            spi_2.exit_close_loop();
            init_motor_set_q = _ctrlComp->lowState->getQ_Hex();

            Vec36 motor_set;
            Vec36 motor_kp;
            Vec36 motor_kd;
            for (int i = 0; i < 18; i++)
            {
                _ctrlComp->lowCmd->motorCmd[i].q = init_motor_set_q(i);
                motor_set(i) = _ctrlComp->lowCmd->motorCmd[i].q;
                _ctrlComp->lowCmd->setAllLegGain(30, 1);
            }

            if( _ctrlComp->lowCmd->motorCmd[0].q == 0.000 or _ctrlComp->lowCmd->motorCmd[1].q == 0.000 or _ctrlComp->lowCmd->motorCmd[2].q == 0.000 or
            _ctrlComp->lowCmd->motorCmd[3].q == 0.000 or _ctrlComp->lowCmd->motorCmd[4].q == 0.000 or _ctrlComp->lowCmd->motorCmd[5].q == 0.000 or
            _ctrlComp->lowCmd->motorCmd[6].q == 0.000 or _ctrlComp->lowCmd->motorCmd[7].q == 0.000 or _ctrlComp->lowCmd->motorCmd[8].q == 0.000 or
            _ctrlComp->lowCmd->motorCmd[9].q == 0.000 or _ctrlComp->lowCmd->motorCmd[10].q == 0.000 or _ctrlComp->lowCmd->motorCmd[11].q == 0.000 or
            _ctrlComp->lowCmd->motorCmd[12].q == 0.000 or _ctrlComp->lowCmd->motorCmd[13].q == 0.000 or _ctrlComp->lowCmd->motorCmd[14].q == 0.000 or
            _ctrlComp->lowCmd->motorCmd[15].q == 0.000 or _ctrlComp->lowCmd->motorCmd[16].q == 0.000 or _ctrlComp->lowCmd->motorCmd[17].q == 0.000 )
                get_motor_response_flag = false;
            else
                get_motor_response_flag = true;

            MOTOR_ENTER_CLOSELOOP = false;
            MOTOR_READY_FLAG = false;
            MOTOR_ENABLE_FLAG = false;
            // std::cout<<"\n MOTOR_DISABEL_FLAG : rec_moter_q:  \n"<< spi_2.rec_moter_q* 180/3.1415926 <<std::endl;
            // std::cout<<"\n MOTOR_DISABEL_FLAG : getQ_Hex:  \n"<< _ctrlComp->lowState->getQ_Hex() * 180/3.1415926 <<std::endl;
            std::cout<<"MOTOR_DISABEL_FLAG : motor_set:  \n"<< motor_set * 180/3.1415926 <<std::endl;
        }
        else if( MOTOR_ENABLE_FLAG == true && get_motor_response_flag == true){ // 'p'
            for (int i = 0; i < 6; i++){
                spi_2.enter_close_loop();
                usleep(5000);
            }
            init_motor_set_q = _ctrlComp->lowState->getQ_Hex();
            MOTOR_ENABLE_FLAG = false;
            MOTOR_ENTER_CLOSELOOP = true;
        }
        else if( MOTOR_READY_FLAG == true && MOTOR_ENTER_CLOSELOOP == true){ // '['
            spi_2.send_all_data();
            dou_dong_angle.setZero();
            for (int i = 0; i < 18; i++){
                dou_dong_angle(i) = DOU_DONG_ANGEL;
            }
            _ctrlComp->lowCmd->setQ( vec36ToVec18( init_motor_set_q + dou_dong_angle )  );
        }

        _startTime = getSystemTime();
        _ctrlComp->sendRecv();

        if( MOTOR_DATA_LOAD == true && MOTOR_ENTER_CLOSELOOP == true){ // ']'
            spi_2.send_all_data();
            algorithm_run();
        }
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));

        // std::cout<< " RPY: "<< rotMatToRPY( _ctrlComp->lowState->getRotMat() ).transpose()*180/3.1415926 << std::endl;
        // std::cout<< " getAcc: "<<  _ctrlComp->lowState->getAcc().transpose() << std::endl;
        // std::cout<< " getGyro: "<<  _ctrlComp->lowState->getGyro().transpose() << std::endl;
    #else
        if( _waitCount <= 100 ){
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
        algorithm_run();
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
    #endif
}

void FSM::algorithm_run(){

    if( fsm_first_start == true ){
        printf(" KeyBoard checkCmd:\n 1->PASSIVE_1(***);\n 2->FIXEDSTAND_2(***);\nc->FIXEDSQUAT_c(***);\n 3->FREESTAND_3;\n 4->QP_4(***);\n 5->POSITION_5(***);\n 6->A1MPC_6(Ban);\n 7->POSREFLEX_7(Ban);\n 9->SWING_TEST9\n");
        printf(" TERRIANESTI_FOURLEG: %d \n",TERRIANESTI_FOURLEG);
        initialize();
    }
    fsm_first_start = false;

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