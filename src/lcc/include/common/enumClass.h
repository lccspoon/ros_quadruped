/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1
};

// 指的是用户能够控制的几种模式
enum class UserCommand{
    // EXIT,
    NONE,
    //START,      // trotting
    // L2_A,       // fixedStand
    // L2_B,       // passive
    // L2_X,       // freeStand
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    BALANCE_TEST0,       // balanceTest
    SWING_TEST9,       // swingTest
    SETP_TEST8,        // stepTest

    PASSIVE_1,
    FIXEDSTAND_2,
    FREESTAND_3,
    VMC_4,
    TROTTING_5,
    A1MPC_6        //lcc 20240416
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    BALANCETEST,
    SWINGTEST,
    STEPTEST,
    A1MPC,  // lcc 20240416
    VMC  // lcc 20240523
};

#endif  // ENUMCLASS_H