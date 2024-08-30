#ifndef CMDPANEL_H
#define CMDPANEL_H

#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <stdio.h>
#include "message/unitree_joystick.h"
#include "common/enumClass.h"
#include <pthread.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include "control/neural_bezier_curve.h"
// #include "interface/KeyBoard.h"

#ifdef COMPILE_WITH_REAL_ROBOT
    #ifdef ROBOT_TYPE_A1
        #include "unitree_legged_sdk/unitree_legged_sdk.h"
    #endif  // ROBOT_TYPE_A1
    #ifdef ROBOT_TYPE_Go1
        #include "unitree_legged_sdk/unitree_legged_sdk.h"
    #endif  // ROBOT_TYPE_Go1
#endif  // COMPILE_WITH_REAL_ROBOT

// bool USVLCC_SETZERO;

    // int _add_count=0;
    // float _data_last,_set_last;
    // bool _conver_done_flag=true;

// UserValue指用户能够直接控制的输入变量
class UserValue{
public:
    UserValue(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
    }
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    void setZero(){
        lx = 0;
        ly = 0;
        rx = 0;
        ry = 0;
        L2 = 0;
    }
};

class UserValue_lcc
{
private:
    /* data */
    linear_trans lt_lx;
    linear_trans lt_ly;
    linear_trans lt_l2;
    linear_trans lt_rx;
    linear_trans lt_ry;
public:
    float lx;
    float ly;
    float rx;
    float ry;
    float L2;
    UserValue_lcc(/* args */);
    ~UserValue_lcc();

    void setZero();

};

extern UserValue_lcc userValue_lcc;


/*
 @author:lcc
 @date: 20250601
*/
struct UserFunctionMode{
    bool function_test;
    bool state_reset;
    Eigen::Matrix<double,1,6>  LEG_LIFT_TRIGGER;//20230907cheet
    Eigen::Matrix<double,1,6>  LEG_DOWM_TRIGGER;//20230907cheet
    Eigen::Matrix<double,1,6>  LEG_MKAN_TRIGGER;//20230907cheet
    int life_reaction_off_on=0,dowm_reaction_off_on=0,mkan_reaction_off_on=0,berzier_shape_off_on = 0;
    double set_pitch;

    bool motor_enable_flag; // 进入闭环
    bool motor_disenable_flag;//退出闭环

    UserFunctionMode(){
        setZero();
    }
    void setZero(){
        function_test = false;
        state_reset = false;
        LEG_LIFT_TRIGGER.setZero();
        LEG_DOWM_TRIGGER.setZero();
        LEG_MKAN_TRIGGER.setZero();
        set_pitch = 0;

        motor_enable_flag = false;
        motor_disenable_flag = false;
    }
};

class CmdPanel{
public:
    CmdPanel(){}
    virtual ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    UserFunctionMode getUserFunctionMode(){return userFunctionMode;}
    void setPassive(){userCmd = UserCommand::PASSIVE_1;}
    void setZero(){userValue.setZero();}

    UserFunctionMode userFunctionMode; // lcc 20250601
    // UserValue_lcc userValue_lcc;
#ifdef COMPILE_WITH_REAL_ROBOT
    virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};
#endif  // COMPILE_WITH_REAL_ROBOT
protected:
    virtual void* run(void *arg){return NULL;}
    UserCommand userCmd;
    UserValue userValue;
};

#endif  // CMDPANEL_H