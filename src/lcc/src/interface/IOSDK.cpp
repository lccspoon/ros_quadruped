#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>



//IOSDK::IOSDK():IOInterface(){} 表示在构造 IOSDK 类的对象时，首先调用其基类 IOInterface 的默认构造函数。
//这是一种初始化基类的常见方式，尤其是当基类有一个显式的构造函数时。
IOSDK::IOSDK():IOInterface(){
    std::cout << "The control interface for Real Robot" << std::endl;


    // 由于 KeyBoard 类继承自 CmdPanel 类，因此 KeyBoard 对象也被视为一种 CmdPanel 对象。
    // 这就是继承的基本概念，子类对象可以赋值给父类指针或引用。
    cmdPanel = new KeyBoard();
}

IOSDK::~IOSDK(){
    delete cmdPanel;
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    state->userFunctionMode = cmdPanel->getUserFunctionMode();// lcc 20250601
    // state->userFunctionMode_p->function_test = cmdPanel->getUserFunctionMode().function_test;
    // state->userFunctionMode_p = &cmdPanel->getUserFunctionMode();
    // retSimOdeBodyP();
    // retSimOdeBodyV();
    // ODE_P = retSimOdeBodyP();
    // ODE_V = retSimOdeBodyV();
    // std::cout << "retSimOdeBodyP:\n" <<retSimOdeBodyP()<< std::endl;
    // std::cout << "retSimOdeBodyV:\n" <<retSimOdeBodyV()<< std::endl;

}


void IOSDK::sendCmd(const LowlevelCmd *lowCmd){

}

void IOSDK::recvState(LowlevelState *state){
}