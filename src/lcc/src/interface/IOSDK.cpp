#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"
#include "interface/imu.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

#if USE_A_REAL_HEXAPOD == true
spi_sr spi_2;
bool MOTOR_DISABEL_FLAG = false;
bool MOTOR_ENABLE_FLAG = false;
bool MOTOR_READY_FLAG = false;
bool MOTOR_DATA_LOAD = false;
bool MOTOR_ENTER_CLOSELOOP = false;
float DOU_DONG_ANGEL = 0;
bool TEST_FLAG = false;
Vec36 rec_offset;
#endif

//IOSDK::IOSDK():IOInterface(){} 表示在构造 IOSDK 类的对象时，首先调用其基类 IOInterface 的默认构造函数。
//这是一种初始化基类的常见方式，尤其是当基类有一个显式的构造函数时。
IOSDK::IOSDK():IOInterface(){
    std::cout << "The control interface for Real Robot" << std::endl;

    // 由于 KeyBoard 类继承自 CmdPanel 类，因此 KeyBoard 对象也被视为一种 CmdPanel 对象。
    // 这就是继承的基本概念，子类对象可以赋值给父类指针或引用。
    cmdPanel = new KeyBoard();

//-------------20240814 lcc---------------//
// MOTOR_ENABLE_FLAG : getQ_Hex:  
//   -0.4481   64.4670   -1.6502    6.0216    1.9344    2.3278
//  -54.3473   95.5039 -119.5684 -124.1146 -127.1090   83.7667
//  102.5637 -134.8901  -47.3530   82.6302 -115.9620  110.3448

//-------------20240814 lcc---------------//
// MOTOR_ENABLE_FLAG : getQ_Hex:  
//   -0.4481   64.4670   -1.6283    6.0216    1.9562    2.3715
//  -54.3691   95.5258 -119.5684 -124.1365 -127.1309   83.7448
//  102.5419 -134.9120  -47.3093   82.6520 -115.9183  110.3448

    #if USE_A_REAL_HEXAPOD == true
    float rad2;
    rad2 = 3.1415926/180;
    rec_offset(0) = -0.4481 * rad2;
    rec_offset(1) = -54.3691 * rad2;
    // rec_offset(2) = 102.5419 * rad2;
    rec_offset(2) = 65.7134 * rad2;

    rec_offset(3) = 64.4670 * rad2;
    rec_offset(4) = 95.5258 * rad2;
    // rec_offset(5) = -134.9120 * rad2;
    rec_offset(5) = -87.8731 * rad2;

    rec_offset(6) = -1.6283 * rad2;
    rec_offset(7) = -119.5684 * rad2;
    // rec_offset(8) = -47.3093 * rad2;
    rec_offset(8) = -31.0756 * rad2;

    rec_offset(9) = 6.0216 * rad2;
    rec_offset(10) = -124.1365 * rad2;
    // rec_offset(11) = 82.6520 * rad2;
    rec_offset(11) = 53.4180 * rad2;

    rec_offset(12) = 1.9562 * rad2;
    rec_offset(13) = -127.1309 * rad2;
    // rec_offset(14) = -115.9183 * rad2;
    rec_offset(14) = -76.0134 * rad2;

    rec_offset(15) = 2.3715 * rad2;
    rec_offset(16) = 83.7448 * rad2;
    // rec_offset(17) = 110.3448 * rad2;
    rec_offset(17) = 70.7158 * rad2;
    #endif
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
    #if USE_A_REAL_HEXAPOD == true
    Vec3 rf_q, rf_t;
    Vec3 lf_q, lf_t;
    Vec3 rm_q, rm_t;
    Vec3 lm_q, lm_t;
    Vec3 rb_q, rb_t;
    Vec3 lb_q, lb_t;
    rf_q << lowCmd->motorCmd[0].q, lowCmd->motorCmd[1].q, lowCmd->motorCmd[2].q;
    lf_q << lowCmd->motorCmd[3].q, lowCmd->motorCmd[4].q, lowCmd->motorCmd[5].q;
    rm_q << lowCmd->motorCmd[6].q, lowCmd->motorCmd[7].q, lowCmd->motorCmd[8].q;
    lm_q << lowCmd->motorCmd[9].q, lowCmd->motorCmd[10].q, lowCmd->motorCmd[11].q;
    rb_q << lowCmd->motorCmd[12].q, lowCmd->motorCmd[13].q, lowCmd->motorCmd[14].q;
    lb_q << lowCmd->motorCmd[15].q, lowCmd->motorCmd[16].q, lowCmd->motorCmd[17].q;

    rf_t << lowCmd->motorCmd[0].tau, lowCmd->motorCmd[1].tau, lowCmd->motorCmd[2].tau;
    lf_t << lowCmd->motorCmd[3].tau, lowCmd->motorCmd[4].tau, lowCmd->motorCmd[5].tau;
    rm_t << lowCmd->motorCmd[6].tau, lowCmd->motorCmd[7].tau, lowCmd->motorCmd[8].tau;
    lm_t << lowCmd->motorCmd[9].tau, lowCmd->motorCmd[10].tau, lowCmd->motorCmd[11].tau;
    rb_t << lowCmd->motorCmd[12].tau, lowCmd->motorCmd[13].tau, lowCmd->motorCmd[14].tau;
    lb_t << lowCmd->motorCmd[15].tau, lowCmd->motorCmd[16].tau, lowCmd->motorCmd[17].tau;

    double radd;
    radd = 3.1415926/180;

    // std::cout<<"\n rm_q:  \n"<< rm_q * 180/3.1415926 <<std::endl;
    if( rm_q(1)<=-39 * radd ) { rm_q(1) = -39 * radd; //printf("\n rm_q(1) exce limit! \n "); 
    }
    if( lm_q(1)<=-39 * radd ) { lm_q(1) = -39 * radd; //printf("\n lm_q(1) exce limit! \n "); 
    }
    // std::cout<<"\n rm_q after:  \n"<< rm_q * 180/3.1415926 <<std::endl;

    // Vec36 torq;
    // for (int i = 0; i < 18; i++)
    //     torq(i) = lowCmd->motorCmd[i].tau;
    // std::cout<<"\n torq torq:  \n"<< torq <<std::endl;

    if( MOTOR_ENTER_CLOSELOOP == true)
    {
        rf_q(1) = -rf_q(1);
        rf_q(2) = -rf_q(2);

        lf_q(0) = -lf_q(0);

        rm_q(1) = -rm_q(1);

        lm_q(0) = -lm_q(0);
        lm_q(2) = -lm_q(2);

        lb_q(0) = -lb_q(0);
        lb_q(1) = -lb_q(1);
        lb_q(2) = -lb_q(2);

        // t
        rf_t(1) = -rf_t(1);
        rf_t(2) = -rf_t(2);

        lf_t(0) = -lf_t(0);

        rm_t(1) = -rm_t(1);

        lm_t(0) = -lm_t(0);
        lm_t(2) = -lm_t(2);

        lb_t(0) = -lb_t(0);
        lb_t(1) = -lb_t(1);
        lb_t(2) = -lb_t(2);

        rf_q = rf_q + rec_offset.col(0);
        lf_q = lf_q + rec_offset.col(1);
        rm_q = rm_q + rec_offset.col(2);
        lm_q = lm_q + rec_offset.col(3);
        rb_q = rb_q + rec_offset.col(4);
        lb_q = lb_q + rec_offset.col(5);

        rf_q(2) = ( rf_q(2) + 0 ) / 0.6429;
        lf_q(2) = ( lf_q(2) + 0 ) / 0.6429;
        rm_q(2) = ( rm_q(2) + 0 ) / 0.6429;
        lm_q(2) = ( lm_q(2) + 0 ) / 0.6429;
        rb_q(2) = ( rb_q(2) + 0 ) / 0.6429;
        lb_q(2) = ( lb_q(2) + 0 ) / 0.6429;

        rf_q = spi_2.___dataUnuProtect[1].sendDataConPro(0,rf_q,20*radd);
        lf_q = spi_2.___dataUnuProtect[2].sendDataConPro(1,lf_q,20*radd);
        rm_q = spi_2.___dataUnuProtect[3].sendDataConPro(2,rm_q,20*radd);
        lm_q = spi_2.___dataUnuProtect[4].sendDataConPro(3,lm_q,20*radd);
        rb_q = spi_2.___dataUnuProtect[5].sendDataConPro(4,rb_q,20*radd);
        lb_q = spi_2.___dataUnuProtect[6].sendDataConPro(5,lb_q,20*radd);

        //  rec_moter_q 是真实返回的电机角度，没有经过任何补偿和修饰
        spi_2.___dataUnuProtect[1].velLimAndDifFroDesPosAndActPos(0,3,
                                                        rf_q, 
                                                        spi_2.rec_moter_q.block<3, 1>(0, 0), 20 * radd,
                                                        spi_2.rec_moter_v.block<3, 1>(0, 0), 9);
        spi_2.___dataUnuProtect[2].velLimAndDifFroDesPosAndActPos(1,3,
                                                        lf_q, 
                                                        spi_2.rec_moter_q.block<3, 1>(0, 1), 20 * radd,
                                                        spi_2.rec_moter_v.block<3, 1>(0, 1), 9);
        spi_2.___dataUnuProtect[3].velLimAndDifFroDesPosAndActPos(2,3,
                                                        rm_q, 
                                                        spi_2.rec_moter_q.block<3, 1>(0, 2), 20 * radd,
                                                        spi_2.rec_moter_v.block<3, 1>(0, 2), 9);
        spi_2.___dataUnuProtect[4].velLimAndDifFroDesPosAndActPos(3,3,
                                                        lm_q, 
                                                        spi_2.rec_moter_q.block<3, 1>(0, 3), 20 * radd,
                                                        spi_2.rec_moter_v.block<3, 1>(0, 3), 9);
        spi_2.___dataUnuProtect[5].velLimAndDifFroDesPosAndActPos(4,3,
                                                        rb_q,
                                                        spi_2.rec_moter_q.block<3, 1>(0, 4), 20 * radd,
                                                        spi_2.rec_moter_v.block<3, 1>(0, 4), 9);
        spi_2.___dataUnuProtect[6].velLimAndDifFroDesPosAndActPos(5,3,
                                                        lb_q, 
                                                        spi_2.rec_moter_q.block<3, 1>(0, 5), 20 * radd,
                                                        spi_2.rec_moter_v.block<3, 1>(0, 5), 9);

        // 如果有false,那么pub_data就不会执行
        if (spi_2.___dataUnuProtect[5].diff_val_flag == false or spi_2.___dataUnuProtect[4].diff_val_flag == false 
        or spi_2.___dataUnuProtect[3].diff_val_flag == false or spi_2.___dataUnuProtect[2].diff_val_flag == false 
        or spi_2.___dataUnuProtect[1].diff_val_flag == false or spi_2.___dataUnuProtect[6].diff_val_flag == false)
        {
                spi_2.___dataUnuProtect[5].diff_val_flag = false;
                spi_2.___dataUnuProtect[4].diff_val_flag = false;
                spi_2.___dataUnuProtect[3].diff_val_flag = false;
                spi_2.___dataUnuProtect[2].diff_val_flag = false;
                spi_2.___dataUnuProtect[1].diff_val_flag = false;
                spi_2.___dataUnuProtect[6].diff_val_flag = false;
                // printf("\n   ------------diff_val_flag:%d --------------\n",spi_2.___dataUnuProtect[5].diff_val_flag);
                // exit(0);
        }
        else if(spi_2.___dataUnuProtect[5].vel_lim_flag==false or spi_2.___dataUnuProtect[4].vel_lim_flag==false 
        or spi_2.___dataUnuProtect[3].vel_lim_flag==false or spi_2.___dataUnuProtect[2].vel_lim_flag==false 
        or spi_2.___dataUnuProtect[1].vel_lim_flag==false or spi_2.___dataUnuProtect[6].vel_lim_flag==false)
        {
                spi_2.___dataUnuProtect[5].vel_lim_flag=false;spi_2.___dataUnuProtect[4].vel_lim_flag=false;
                spi_2.___dataUnuProtect[3].vel_lim_flag=false;spi_2.___dataUnuProtect[2].vel_lim_flag=false;
                spi_2.___dataUnuProtect[1].vel_lim_flag=false;spi_2.___dataUnuProtect[6].vel_lim_flag=false;
                // printf("\n   ------------vel_lim_flag:%d --------------\n",spi_2.___dataUnuProtect[5].vel_lim_flag);
                // exit(0);
        }
        else
        {
            spi_2.rf1.q = rf_q(0);
            spi_2.rf2.q = rf_q(1);
            spi_2.rf3.q = rf_q(2);
            // spi_2.rf3.q = rf_q(2) / 0.6429;
            // spi_2.rf1.q = lowCmd->motorCmd[0].q;
            // spi_2.rf2.q = lowCmd->motorCmd[1].q;
            // spi_2.rf3.q = lowCmd->motorCmd[2].q;
            // spi_2.rf1.tau = lowCmd->motorCmd[0].tau;
            // spi_2.rf2.tau = lowCmd->motorCmd[1].tau;
            // spi_2.rf3.tau = lowCmd->motorCmd[2].tau;
            spi_2.rf1.tau = rf_t(0);
            spi_2.rf2.tau = rf_t(1);
            spi_2.rf3.tau = rf_t(2);
            spi_2.rf1.Kp = lowCmd->motorCmd[0].Kp;
            spi_2.rf2.Kp = lowCmd->motorCmd[1].Kp;
            spi_2.rf3.Kp = lowCmd->motorCmd[2].Kp;
            spi_2.rf1.Kd = lowCmd->motorCmd[0].Kd;
            spi_2.rf2.Kd = lowCmd->motorCmd[1].Kd;
            spi_2.rf3.Kd = lowCmd->motorCmd[2].Kd;

            spi_2.lf1.q = lf_q(0);
            spi_2.lf2.q = lf_q(1);
            spi_2.lf3.q = lf_q(2);
            // spi_2.lf3.q = lf_q(2) / 0.6429;
            // spi_2.lf1.q = lowCmd->motorCmd[3].q;
            // spi_2.lf2.q = lowCmd->motorCmd[4].q;
            // spi_2.lf3.q = lowCmd->motorCmd[5].q;
            // spi_2.lf1.tau = lowCmd->motorCmd[3].tau;
            // spi_2.lf2.tau = lowCmd->motorCmd[4].tau;
            // spi_2.lf3.tau = lowCmd->motorCmd[5].tau;
            spi_2.lf1.tau = lf_t(0);
            spi_2.lf2.tau = lf_t(1);
            spi_2.lf3.tau = lf_t(2);
            spi_2.lf1.Kp = lowCmd->motorCmd[3].Kp;
            spi_2.lf2.Kp = lowCmd->motorCmd[4].Kp;
            spi_2.lf3.Kp = lowCmd->motorCmd[5].Kp;
            spi_2.lf1.Kd = lowCmd->motorCmd[3].Kd;
            spi_2.lf2.Kd = lowCmd->motorCmd[4].Kd;
            spi_2.lf3.Kd = lowCmd->motorCmd[5].Kd;

            spi_2.rm1.q = rm_q(0);
            spi_2.rm2.q = rm_q(1);
            spi_2.rm3.q = rm_q(2);
            // spi_2.rm3.q = rm_q(2) / 0.6429;
            // spi_2.rm1.q = lowCmd->motorCmd[6].q;
            // spi_2.rm2.q = lowCmd->motorCmd[7].q;
            // spi_2.rm3.q = lowCmd->motorCmd[8].q;
            // spi_2.rm1.tau = lowCmd->motorCmd[6].tau;
            // spi_2.rm2.tau = lowCmd->motorCmd[7].tau;
            // spi_2.rm3.tau = lowCmd->motorCmd[8].tau;
            spi_2.rm1.tau = rm_t(0);
            spi_2.rm2.tau = rm_t(1);
            spi_2.rm3.tau = rm_t(2);
            spi_2.rm1.Kp = lowCmd->motorCmd[6].Kp;
            spi_2.rm2.Kp = lowCmd->motorCmd[7].Kp;
            spi_2.rm3.Kp = lowCmd->motorCmd[8].Kp;
            spi_2.rm1.Kd = lowCmd->motorCmd[6].Kd;
            spi_2.rm2.Kd = lowCmd->motorCmd[7].Kd;
            spi_2.rm3.Kd = lowCmd->motorCmd[8].Kd;

            spi_2.lm1.q = lm_q(0);
            spi_2.lm2.q = lm_q(1);
            spi_2.lm3.q = lm_q(2);
            // spi_2.lm1.q = lowCmd->motorCmd[9].q;
            // spi_2.lm2.q = lowCmd->motorCmd[10].q;
            // spi_2.lm3.q = lowCmd->motorCmd[11].q;
            // spi_2.lm1.tau = lowCmd->motorCmd[9].tau;
            // spi_2.lm2.tau = lowCmd->motorCmd[10].tau;
            // spi_2.lm3.tau = lowCmd->motorCmd[11].tau;
            spi_2.lm1.tau = lm_t(0);
            spi_2.lm2.tau = lm_t(1);
            spi_2.lm3.tau = lm_t(2);
            spi_2.lm1.Kp = lowCmd->motorCmd[9].Kp;
            spi_2.lm2.Kp = lowCmd->motorCmd[10].Kp;
            spi_2.lm3.Kp = lowCmd->motorCmd[11].Kp;
            spi_2.lm1.Kd = lowCmd->motorCmd[9].Kd;
            spi_2.lm2.Kd = lowCmd->motorCmd[10].Kd;
            spi_2.lm3.Kd = lowCmd->motorCmd[11].Kd;

            spi_2.rb1.q = rb_q(0);
            spi_2.rb2.q = rb_q(1);
            spi_2.rb3.q = rb_q(2);
            // spi_2.rb3.q = rb_q(2) / 0.6429;
            // spi_2.rb1.q = lowCmd->motorCmd[12].q;
            // spi_2.rb2.q = lowCmd->motorCmd[13].q;
            // spi_2.rb3.q = lowCmd->motorCmd[14].q;
            // spi_2.rb1.tau = lowCmd->motorCmd[12].tau;
            // spi_2.rb2.tau = lowCmd->motorCmd[13].tau;
            // spi_2.rb3.tau = lowCmd->motorCmd[14].tau;
            spi_2.rb1.tau = rb_t(0);
            spi_2.rb2.tau = rb_t(1);
            spi_2.rb3.tau = rb_t(2);
            spi_2.rb1.Kp = lowCmd->motorCmd[12].Kp;
            spi_2.rb2.Kp = lowCmd->motorCmd[13].Kp;
            spi_2.rb3.Kp = lowCmd->motorCmd[14].Kp;
            spi_2.rb1.Kd = lowCmd->motorCmd[12].Kd;
            spi_2.rb2.Kd = lowCmd->motorCmd[13].Kd;
            spi_2.rb3.Kd = lowCmd->motorCmd[14].Kd;

            spi_2.lb1.q = lb_q(0);
            spi_2.lb2.q = lb_q(1);
            spi_2.lb3.q = lb_q(2);
            // spi_2.lb3.q = lb_q(2) / 0.6429;
            // spi_2.lb1.q = lowCmd->motorCmd[15].q;
            // spi_2.lb2.q = lowCmd->motorCmd[16].q;
            // spi_2.lb3.q = lowCmd->motorCmd[17].q;
            // spi_2.lb1.tau = lowCmd->motorCmd[15].tau;
            // spi_2.lb2.tau = lowCmd->motorCmd[16].tau;
            // spi_2.lb3.tau = lowCmd->motorCmd[17].tau;
            spi_2.lb1.tau = lb_t(0);
            spi_2.lb2.tau = lb_t(1);
            spi_2.lb3.tau = lb_t(2);
            spi_2.lb1.Kp = lowCmd->motorCmd[15].Kp;
            spi_2.lb2.Kp = lowCmd->motorCmd[16].Kp;
            spi_2.lb3.Kp = lowCmd->motorCmd[17].Kp;
            spi_2.lb1.Kd = lowCmd->motorCmd[15].Kd;
            spi_2.lb2.Kd = lowCmd->motorCmd[16].Kd;
            spi_2.lb3.Kd = lowCmd->motorCmd[17].Kd;

            // printf("\n ------------ sending ------------ \n");
        }
    }
    #endif
}

void IOSDK::recvState(LowlevelState *state){

    // Vec36 rec_moter_q;
    // Vec36 rec_moter_v;
    // Vec36 rec_moter_t;

    // Vec36 rec_moter_q_last;
    // Vec36 rec_moter_v_last;
    // Vec36 rec_moter_t_last;

    // Vec36 rec_moter_q_erroCount;
    // Vec36 rec_moter_v_erroCount;
    // Vec36 rec_moter_t_erroCount;

    #if USE_A_REAL_HEXAPOD == true

        for(int i(0); i < NUM_DOF_W; ++i){
            spi_2.rec_moter_q(i) = spi_2.motor_states[i].q;
            spi_2.rec_moter_v(i) = spi_2.motor_states[i].dq;
            spi_2.rec_moter_t(i) = spi_2.motor_states[i].tau;

            // if( TEST_FLAG == true  && i == 2)
            //     spi_2.rec_moter_q(i) = 0;

            if(MOTOR_ENTER_CLOSELOOP == true){
                if( spi_2.rec_moter_q(i) == 0.0000){
                    spi_2.rec_moter_q(i) = spi_2.rec_moter_q_last(i);
                    spi_2.rec_moter_v(i) = spi_2.rec_moter_v_last(i);
                    spi_2.rec_moter_t(i) = spi_2.rec_moter_t_last(i);
                    spi_2.rec_moter_q_erroCount(i) += 1;
                    spi_2.rec_moter_v_erroCount(i) += 1;
                    spi_2.rec_moter_t_erroCount(i) += 1;
                    std::cout<<"\n warning : rec_moter_q_erroCount:  \n"<< spi_2.rec_moter_q_erroCount <<std::endl;
                }
                else{
                    spi_2.rec_moter_q_last(i) = spi_2.rec_moter_q(i);
                    spi_2.rec_moter_v_last(i) = spi_2.rec_moter_v(i);
                    spi_2.rec_moter_t_last(i) = spi_2.rec_moter_t(i);
                }
            }
        }

        for(int i(0); i < NUM_DOF_W; ++i){

            if( i == 2 || i == 5 || i == 8 || i == 11 || i == 14 || i == 17)
                state->motorState[i].q = spi_2.rec_moter_q(i) * 0.6429 - rec_offset(i);
            else
                state->motorState[i].q = spi_2.rec_moter_q(i) - rec_offset(i);
                
            state->motorState[i].dq = spi_2.rec_moter_v(i);
            state->motorState[i].tauEst = spi_2.rec_moter_t(i);
        }

        // q校正
        state->motorState[1].q = -state->motorState[1].q;
        state->motorState[2].q = -state->motorState[2].q;

        state->motorState[7].q = -state->motorState[7].q;

        state->motorState[11].q = -state->motorState[11].q;

        state->motorState[16].q = -state->motorState[16].q;
        state->motorState[17].q = -state->motorState[17].q;

        state->motorState[3].q = -state->motorState[3].q;
        state->motorState[9].q = -state->motorState[9].q;
        state->motorState[15].q = -state->motorState[15].q;

        // t校正
        state->motorState[1].tauEst = -state->motorState[1].tauEst;
        state->motorState[2].tauEst = -state->motorState[2].tauEst;

        state->motorState[7].tauEst = -state->motorState[7].tauEst;

        state->motorState[11].tauEst = -state->motorState[11].tauEst;

        state->motorState[16].tauEst = -state->motorState[16].tauEst;
        state->motorState[17].tauEst = -state->motorState[17].tauEst;

        state->motorState[3].tauEst = -state->motorState[3].tauEst;
        state->motorState[9].tauEst = -state->motorState[9].tauEst;
        state->motorState[15].tauEst = -state->motorState[15].tauEst;

        // dq校正
        state->motorState[1].dq = -state->motorState[1].dq;
        state->motorState[2].dq = -state->motorState[2].dq;

        state->motorState[7].dq = -state->motorState[7].dq;

        state->motorState[11].dq = -state->motorState[11].dq;

        state->motorState[16].dq = -state->motorState[16].dq;
        state->motorState[17].dq = -state->motorState[17].dq;

        state->motorState[3].dq = -state->motorState[3].dq;
        state->motorState[9].dq = -state->motorState[9].dq;
        state->motorState[15].dq = -state->motorState[15].dq;

        // state->motorState[2].q = (spi_2.rec_moter_q(2) - rec_offset(2)) * 0.6429 ;
        // state->motorState[5].q = (spi_2.rec_moter_q(5) - rec_offset(5)) * 0.6429 ;
        // state->motorState[8].q = (spi_2.rec_moter_q(8) - rec_offset(8)) * 0.6429 ;
        // state->motorState[11].q = (spi_2.rec_moter_q(11) - rec_offset(11)) * 0.6429 ;
        // state->motorState[14].q = (spi_2.rec_moter_q(14) - rec_offset(14)) * 0.6429 ;
        // state->motorState[17].q = (spi_2.rec_moter_q(17) - rec_offset(17)) * 0.6429 ;

        // std::cout<<"MOTOR_DISABEL_FLAG : rec_moter_q:  \n"<< spi_2.rec_moter_q  * 180/3.1415926<<std::endl;
        // std::cout<<"MOTOR_DISABEL_FLAG : rec_moter_q_last:  \n"<< spi_2.rec_moter_q_last  * 180/3.1415926<<std::endl;
        // std::cout<<"MOTOR_DISABEL_FLAG : rec_moter_q_erroCount:  \n"<< spi_2.rec_moter_q_erroCount <<std::endl;

        for(int i(0); i < 3; ++i){
            state->imu.accelerometer[i] = hipnuc_raw.hi91.acc[i]*GRAVITY;
            state->imu.gyroscope[i] = hipnuc_raw.hi91.gyr[i];
        }

        // Note: state->imu.quaternion:  w, x, y, z
        //   geometry_msgs/Quaternion orientation
        //   float64 x
        //   float64 y
        //   float64 z
        //   float64 w

        state->imu.quaternion[0] = hipnuc_raw.hi91.quat[0];
        state->imu.quaternion[1] = hipnuc_raw.hi91.quat[1];
        state->imu.quaternion[2] = hipnuc_raw.hi91.quat[2];
        state->imu.quaternion[3] = hipnuc_raw.hi91.quat[3];

        // printf(" roll:%f,  pitch:%f, yaw:%f\n",hipnuc_raw.hi91.roll, hipnuc_raw.hi91.pitch, hipnuc_raw.hi91.yaw);
        // printf(" acc0:%f,  acc1:%f, acc2:%f\n",hipnuc_raw.hi91.acc[0]*GRAVITY, hipnuc_raw.hi91.acc[1]*GRAVITY, hipnuc_raw.hi91.acc[2]*GRAVITY);
        // printf(" gyr0:%f,  gyr1:%f, gyr2:%f\n",hipnuc_raw.hi91.gyr[0], hipnuc_raw.hi91.gyr[1], hipnuc_raw.hi91.gyr[2]);
        // printf(" quaternion 0:%f,  1:%f, 2:%f 3:%f \n",state->imu.quaternion[0], state->imu.quaternion[1], state->imu.quaternion[2], state->imu.quaternion[3]);

    #endif

}