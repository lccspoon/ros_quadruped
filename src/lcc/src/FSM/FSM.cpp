
#include "FSM/FSM.h"
#include "interface/IOSDK.h"
#include <iostream>
#include "interface/IOROS.h"
#include "interface/imu.h"
#include <thread>
#include "interface/KeyBoard.h"
#include "interface/CmdPanel.h"
#include <fstream>
#include "control/TerrianEsti.h"

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
    _stateList.force_pos = new State_Force_Pos(_ctrlComp);//lcc 20240827
    _stateList.mpc_force_pos = new State_MPC_Force_Pos(_ctrlComp);//lcc 20240827
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

    #if TXT_FLAGE
            motor_t.open("./motor_t.txt", ios::out | ios::trunc);
            // motor_p.open("./motor_p.txt", ios::out | ios::trunc);

            robot_RPY.open("./robot_RPY.txt", ios::out | ios::trunc);
            // robot_pos_world.open("./robot_pos_world.txt", ios::out | ios::trunc);
            // robot_vel_world.open("./robot_vel_world.txt", ios::out | ios::trunc);

            // feet_force_des.open("./feet_force_des.txt", ios::out | ios::trunc);
            feet_force_est.open("./feet_force_est.txt", ios::out | ios::trunc);
            feet_pos_world.open("./feet_pos_world.txt", ios::out | ios::trunc);
    #endif
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
                _ctrlComp->lowCmd->setAllLegGain(100, 1);
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
            // printf(" _yaw: %f \n", _ctrlComp->lowState->getYaw()*180/3.1415926);
            std::cout<<"rotMatToRPY:"<< rotMatToRPY(_ctrlComp->lowState->getRotMat()).transpose()*180/3.1415926 <<std::endl;
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
        else{
            // printf(" _yaw: %f \n", _ctrlComp->lowState->getYaw()*180/3.1415926);
            // std::cout<<"rotMatToRPY:"<< rotMatToRPY(_ctrlComp->lowState->getRotMat()).transpose()*180/3.1415926 <<std::endl;
        }

        _startTime = getSystemTime();
        _ctrlComp->sendRecv();

        if( MOTOR_DATA_LOAD == true && MOTOR_ENTER_CLOSELOOP == true){ // ']'
            spi_2.send_all_data();
            fsm_run();
            // data_save2txt();
        }
        else{
            // printf(" _yaw: %f \n", _ctrlComp->lowState->getYaw()*180/3.1415926);
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
        fsm_run();
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
    #endif
}

void FSM::fsm_run(){

    if( USVLCC_SETZERO == true ){  
        userValue_lcc.setZero();
    }

    if( fsm_first_start == true ){
        #if USE_A_REAL_HEXAPOD == true
        printf(" KeyBoard checkCmd:\n 1->PASSIVE_1 ( ----******---- );\n 2->FIXEDSTAND_2 ( ----******---- );\n c->FIXEDSQUAT_c ( ----******---- );\n 3->FREESTAND_3;\n 4->QP_4 ( ----******---- );\n 5->POSITION_5 ( ----******---- );\n 6->A1MPC_6(Ban);\n 7->POSREFLEX_7(Ban);\n 8->FORCE_POS ( ----******---- );\n 9->SWING_TEST9\n; 0->MPC_FOREC_POS0 ( ----******---- )\n");
        printf(" TERRIANESTI_FOURLEG: %d \n",TERRIANESTI_FOURLEG);
        #endif
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
    case FSMStateName::FORCE_POS:    //lcc 20240827
        return _stateList.force_pos;
        break;
    case FSMStateName::MPC_FORCE_POS:    //lcc 20240903
        return _stateList.mpc_force_pos;
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

void FSM::data_save2txt(){
    #if TXT_FLAGE
    if( DTAT_SAVE2TXT == true ){
        if ( ! motor_t) { cout << "motor_t 文件不能打开" <<endl; }
            motor_t <<  
            _ctrlComp->lowCmd->motorCmd[0].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[1].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[2].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[3].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[4].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[5].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[6].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[7].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[8].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[9].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[10].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[11].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[12].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[13].tau <<" "<< 
            _ctrlComp->lowCmd->motorCmd[14].tau << "\n";
            // _ctrlComp->lowCmd->motorCmd[15].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[16].tau <<" "<< 
            // _ctrlComp->lowCmd->motorCmd[17].tau << "\n";
        // if ( ! motor_p) { cout << "motor_p 文件不能打开" <<endl; }
        //     motor_p <<  
        //     _ctrlComp->lowCmd->motorCmd[0].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[1].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[2].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[3].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[4].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[5].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[6].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[7].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[8].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[9].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[11].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[12].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[13].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[14].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[15].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[16].q <<" "<< 
        //     _ctrlComp->lowCmd->motorCmd[17].q << "\n";

        Vec3 RPY;    
        RPY = rotMatToRPY(_ctrlComp->lowState->getRotMat());
        Vec3 vel, pos;    
        pos = _ctrlComp->estimator->getPosition();
        vel = _ctrlComp->estimator->getVelocity();
        if ( ! robot_RPY) { cout << "robot_RPY 文件不能打开" <<endl; }
            robot_RPY <<  
            RPY(0) <<" "<< 
            RPY(1) <<" "<< 
            RPY(2) <<" "<< 
            pos(0) <<" "<< 
            pos(1) <<" "<< 
            pos(2) <<" "<< 
            TERRIAN_EST_DEGREE(0) <<" "<< 
            TERRIAN_EST_DEGREE(1) <<" "<< 
            vel(0) <<" "<< 
            vel(1) <<" "<< 
            vel(2) << "\n";

        // if ( ! robot_pos_world) { cout << "robot_pos_world 文件不能打开" <<endl; }
        //     robot_pos_world <<  
        //     pos(0) <<" "<< 
        //     pos(1) <<" "<< 
        //     pos(2) <<" "<< 
        //     TERRIAN_EST_DEGREE(0) <<" "<< 
        //     TERRIAN_EST_DEGREE(1) << "\n";
        // if ( ! robot_vel_world) { cout << "robot_vel_world 文件不能打开" <<endl; }
        //     robot_vel_world <<  
        //     vel(0) <<" "<< 
        //     vel(1) <<" "<< 
            // vel(2) << "\n";

        Vec36 feet_pos;
        feet_pos = _ctrlComp->estimator->getFeetPos();
        if ( ! feet_pos_world) { cout << "feet_pos_world 文件不能打开" <<endl; }
            feet_pos_world <<  
            feet_pos(0) <<" "<< 
            feet_pos(1) <<" "<< 
            feet_pos(2) <<" "<< 
            // feet_pos(3) <<" "<< 
            // feet_pos(4) <<" "<< 
            // feet_pos(5) <<" "<< 
            // feet_pos(6) <<" "<< 
            // feet_pos(7) <<" "<< 
            // feet_pos(8) <<" "<< 
            feet_pos(9) <<" "<< 
            feet_pos(10) <<" "<< 
            feet_pos(11) <<" "<< 
            feet_pos(12) <<" "<< 
            feet_pos(13) <<" "<< 
            feet_pos(14) << "\n";
            // feet_pos(15) <<" "<< 
            // feet_pos(16) <<" "<< 
            // feet_pos(17) << "\n";
        Vec36 _footTipForceEst;
        _footTipForceEst = _ctrlComp->sixlegdogModel->calcForceByTauEst( vec36ToVec18(_ctrlComp->lowState->getQ_Hex()), _ctrlComp->lowState->getTau_Hex());
        if ( ! feet_force_est) { cout << "feet_force_est 文件不能打开" <<endl; }
            feet_force_est <<  
            _footTipForceEst(0) <<" "<< 
            _footTipForceEst(1) <<" "<< 
            _footTipForceEst(2) <<" "<< 
            // _footTipForceEst(3) <<" "<< 
            // _footTipForceEst(4) <<" "<< 
            // _footTipForceEst(5) <<" "<< 
            // _footTipForceEst(6) <<" "<< 
            // _footTipForceEst(7) <<" "<< 
            // _footTipForceEst(8) <<" "<< 
            _footTipForceEst(9) <<" "<< 
            _footTipForceEst(10) <<" "<< 
            _footTipForceEst(11) <<" "<< 
            _footTipForceEst(12) <<" "<< 
            _footTipForceEst(13) <<" "<< 
            _footTipForceEst(14) << "\n";
            // _footTipForceEst(15) <<" "<< 
            // _footTipForceEst(16) <<" "<< 
            // _footTipForceEst(17) << "\n";
    }
    #endif
}