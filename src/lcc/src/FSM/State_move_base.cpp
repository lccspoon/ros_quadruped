 
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Position(ctrlComp){
    // _stateName = FSMStateName::MOVE_BASE;
    // _stateNameString = "move_base";
    // initRecv();
}

FSMStateName State_move_base::checkChange(){
    // if(_lowState->userCmd == UserCommand::PASSIVE_1){
    //     return FSMStateName::PASSIVE;
    // }
    // else if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
    //     return FSMStateName::FIXEDSTAND;
    // }
    // else{
    //     return FSMStateName::MOVE_BASE;
    // }
}

void State_move_base::getUserCmd(){
    // setHighCmd(_vx, _vy, _wz);
    // ros::spinOnce();
}

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
    // _vx = msg.linear.x;
    // _vy = msg.linear.y;
    // _wz = msg.angular.z;
}

void State_move_base::initRecv(){
    // _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_move_base::twistCallback, this);
}

#endif  // COMPILE_WITH_MOVE_BASE