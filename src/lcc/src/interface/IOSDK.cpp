#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

#if USE_A_REAL_HEXAPOD == true
spi_sr spi;
#endif

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
    #if USE_A_REAL_HEXAPOD == true
        for(int i(0); i < NUM_DOF_W; ++i){
            state->motorState[i].q = spi.motor_states[i].q;
            state->motorState[i].dq = spi.motor_states[i].dq;
            state->motorState[i].tauEst = spi.motor_states[i].tau;
        }
        for(int i(0); i < 3; ++i){
            // state->imu.accelerometer[i] = sub_imu_lin_a_local(i);
            // state->imu.gyroscope[i] = sub_imu_ang_v_local(i);
        }
        // Note: state->imu.quaternion:  w, x, y, z
        //   geometry_msgs/Quaternion orientation
        //   float64 x
        //   float64 y
        //   float64 z
        //   float64 w

        // state->imu.quaternion[0] = sub_imu_orie_local[3];
        // state->imu.quaternion[1] = sub_imu_orie_local[0];
        // state->imu.quaternion[2] = sub_imu_orie_local[1];
        // state->imu.quaternion[3] = sub_imu_orie_local[2];
    #endif

}