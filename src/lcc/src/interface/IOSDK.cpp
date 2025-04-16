#include "interface/IOSDK.h"
#include "interface/KeyBoard.h"
#include "interface/imu.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <math.h>
#include <cmath>

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

//-------------20250330 lcc---------------//
//  MOTOR_DISABEL_FLAG : getQ_Hex:  
// -115.4374    0.9726   -1.9344    5.8905    1.4972    2.2622
//  -92.9248  134.1688 -115.6997 -126.8467 -132.1143   87.8321
//  -83.7980  -15.5624  -30.0498   50.8325  -70.3645   68.0881

//-------------20250331 lcc---------------//
// -115.5397   -0.4483   -1.9135    6.9868    2.4164   -0.1203
//  -97.4111  137.9107 -120.0664 -123.1716 -127.4578   84.2466
//   86.1928  -29.8608  -55.0965   84.4434 -117.7703  112.1939

//-------------20250331 lcc---------------//
//  MOTOR_DISABEL_FLAG : getQ_Hex:  
// -115.5030   -0.4918   -1.9344    7.2674    2.3934    1.3223
//  -97.3836  137.8189 -120.0711 -123.1092 -127.3713   84.2257
//   55.3572  -19.0753  -35.4317   54.2752  -75.6761   72.1069

//-------------20250401 lcc---------------//
//  MOTOR_DISABEL_FLAG : getQ_Hex: 
// -115.9183   -0.3388   -1.8032    5.5408    1.0382    2.5682
//  -92.7281  133.4038 -115.5467 -127.6555 -131.6116   88.3130
//   51.3664  -15.8715  -31.1178   50.3828  -72.0788   67.0342

// MOTOR_DISABEL_FLAG : motor_set:  
// -116.4429    4.1200    1.9781    9.3001   12.3601   -5.3440
//  -98.4108  134.2781 -120.6175 -121.9508 -126.1255   82.3897
//   57.4368   62.9873  -35.7970   62.1723  -76.7581   87.2829



    #if USE_A_REAL_HEXAPOD == true
    rec_offset.setZero();

    float rad2;
    rad2 = 3.1415926/180;
    rec_offset(0) = -115.9183 * rad2;
    rec_offset(1) = -92.7281 * rad2;
    rec_offset(2) = 51.3664 * rad2;

    rec_offset(3) = -0.3388 * rad2;
    rec_offset(4) = 133.4038 * rad2;
    rec_offset(5) = -17.1081 * rad2;

    rec_offset(6) = -1.8032  * rad2;
    rec_offset(7) = -115.5467 * rad2;
    rec_offset(8) = -31.1178 * rad2;

    rec_offset(9) = 5.5408 * rad2;
    rec_offset(10) = -127.6555 * rad2;
    rec_offset(11) = 50.3828 * rad2;

    rec_offset(12) = 1.0382 * rad2;
    rec_offset(13) = -131.6116 * rad2;
    rec_offset(14) = -72.0788 * rad2;

    rec_offset(15) = 2.5682 * rad2;
    rec_offset(16) = 88.3130 * rad2;
    rec_offset(17) = 67.0342 * rad2;

    // rec_offset.setZero();
    data_out_limt.setZero();

    #endif
}

IOSDK::~IOSDK(){
    delete cmdPanel;
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){

    sendCmd(cmd,state);
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

void IOSDK::sendCmd(const LowlevelCmd *lowCmd, LowlevelState *state){
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

    // std::cout<<"rf_q \n"<< rf_q.transpose() * 180/3.1415926 <<std::endl;
    // std::cout<<"lf_q \n"<< lf_q.transpose() * 180/3.1415926 <<std::endl;
    // std::cout<<"rm_q \n"<< rm_q.transpose() * 180/3.1415926 <<std::endl;
    // std::cout<<"lm_q \n"<< lm_q.transpose() * 180/3.1415926 <<std::endl;
    // std::cout<<"rb_q \n"<< rb_q.transpose() * 180/3.1415926 <<std::endl;
    // std::cout<<"lb_q \n"<< lb_q.transpose() * 180/3.1415926 <<std::endl;

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

        // //q
        rf_q(0) = -rf_q(0);
        rf_q(2) = -rf_q(2);

        lf_q(0) = -lf_q(0);
        lf_q(2) = -lf_q(2);

        rm_q(0) = -rm_q(0);
        rm_q(1) = -rm_q(1);
        rm_q(2) = -rm_q(2);

        lm_q(0) = -lm_q(0);
        lm_q(1) = -lm_q(1);
        lm_q(2) = -lm_q(2);

        rb_q(0) = -rb_q(0);
        rb_q(2) = -rb_q(2);

        lb_q(0) = -lb_q(0);
        lb_q(2) = -lb_q(2);


        // //t
        rf_t(0) = -rf_t(0);
        rf_t(2) = -rf_t(2);

        lf_t(0) = -lf_t(0);
        lf_t(2) = -lf_t(2);

        rm_t(0) = -rm_t(0);
        rm_t(1) = -rm_t(1);
        rm_t(2) = -rm_t(2);

        lm_t(0) = -lm_t(0);
        lm_t(1) = -lm_t(1);
        lm_t(2) = -lm_t(2);

        rb_t(0) = -rb_t(0);
        rb_t(2) = -rb_t(2);

        lb_t(0) = -lb_t(0);
        lb_t(2) = -lb_t(2);

        /* ----------------里面是固定模板，不可动---------------*/
        // //q
        rf_q(0) = -rf_q(0);
        rf_q(2) = -rf_q(2);

        lf_q(1) = -lf_q(1);

        rm_q(0) = -rm_q(0);
        rm_q(1) = -rm_q(1);

        lm_q(2) = -lm_q(2);

        rb_q(0) = -rb_q(0);
        rb_q(1) = -rb_q(1);

        lb_q(2) = -lb_q(2);
        // t
        rf_t(0) = -rf_t(0);
        rf_t(2) = -rf_t(2);

        lf_t(1) = -lf_t(1);

        rm_t(0) = -rm_t(0);
        rm_t(1) = -rm_t(1);

        lm_t(2) = -lm_t(2);

        rb_t(0) = -rb_t(0);
        rb_t(1) = -rb_t(1);

        lb_t(2) = -lb_t(2);
        /* ----------------里面是固定模板，不可动---------------*/


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

        // if(state->userCmd == UserCommand::FORCE_POS_8){FORCE_PROTECT_CHANGE
        if(FORCE_PROTECT_CHANGE == true){
            c_p = 90; v_p = 120;
            // printf(" ------FORCE_POS_8------ \n");
        }
        else {
            c_p = 45; v_p = 60;
            // printf(" ------UserCommand ELSE------ \n");
        }

        // std::cout<<"bf rf_q \n"<< rf_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"bf lf_q \n"<< lf_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"bf rm_q \n"<< rm_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"bf lm_q \n"<< lm_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"bf rb_q \n"<< rb_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"bf lb_q \n"<< lb_q.transpose() * 180/3.1415926 <<std::endl;

        rf_q = spi_2.___dataUnuProtect[1].sendDataConPro(0,rf_q,c_p*radd);
        lf_q = spi_2.___dataUnuProtect[2].sendDataConPro(1,lf_q,c_p*radd);
        rm_q = spi_2.___dataUnuProtect[3].sendDataConPro(2,rm_q,c_p*radd);
        lm_q = spi_2.___dataUnuProtect[4].sendDataConPro(3,lm_q,c_p*radd);
        rb_q = spi_2.___dataUnuProtect[5].sendDataConPro(4,rb_q,c_p*radd);
        lb_q = spi_2.___dataUnuProtect[6].sendDataConPro(5,lb_q,c_p*radd);

        // std::cout<<"af rf_q \n"<< rf_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"af lf_q \n"<< lf_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"af rm_q \n"<< rm_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"af lm_q \n"<< lm_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"af rb_q \n"<< rb_q.transpose() * 180/3.1415926 <<std::endl;
        // std::cout<<"af lb_q \n"<< lb_q.transpose() * 180/3.1415926 <<std::endl;

        // state->motorState[0].q = -state->motorState[0].q;


        // if( 
        // fabs(rf_q(0) - spi_2.rec_moter_q(0,0)) >= v_p * radd ||
        // fabs(rf_q(1) - spi_2.rec_moter_q(1,0)) >= v_p * radd ||
        // fabs(rf_q(2) - spi_2.rec_moter_q(2,0)) >= v_p * radd ||
        // fabs(lf_q(0) - spi_2.rec_moter_q(0,1)) >= v_p * radd ||
        // fabs(lf_q(1) - spi_2.rec_moter_q(1,1)) >= v_p * radd ||
        // fabs(lf_q(2) - spi_2.rec_moter_q(2,1)) >= v_p * radd ||
        // fabs(rm_q(0) - spi_2.rec_moter_q(0,2)) >= v_p * radd ||
        // fabs(rm_q(1) - spi_2.rec_moter_q(1,2)) >= v_p * radd ||
        // fabs(rm_q(2) - spi_2.rec_moter_q(2,2)) >= v_p * radd ||
        // fabs(lm_q(0) - spi_2.rec_moter_q(0,3)) >= v_p * radd ||
        // fabs(lm_q(1) - spi_2.rec_moter_q(1,3)) >= v_p * radd ||
        // fabs(lm_q(2) - spi_2.rec_moter_q(2,3)) >= v_p * radd ||
        // fabs(rb_q(0) - spi_2.rec_moter_q(0,4)) >= v_p * radd ||
        // fabs(rb_q(1) - spi_2.rec_moter_q(1,4)) >= v_p * radd ||
        // fabs(rb_q(2) - spi_2.rec_moter_q(2,4)) >= v_p * radd ||
        // fabs(lb_q(0) - spi_2.rec_moter_q(0,5)) >= v_p * radd ||
        // fabs(lb_q(1) - spi_2.rec_moter_q(1,5)) >= v_p * radd ||
        // fabs(lb_q(2) - spi_2.rec_moter_q(2,5)) >= v_p * radd        
        // ){
        //     data_out_limt(0)++;
        //     printf("--------------- diff_val_flag: des_q - act_q --------------: %f \n", data_out_limt(0));
        // }
        // else 
        if( 
        fabs(spi_2.rec_moter_q_last(0,0) - spi_2.rec_moter_q(0,0)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(1,0) - spi_2.rec_moter_q(1,0)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(2,0) - spi_2.rec_moter_q(2,0)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(0,1) - spi_2.rec_moter_q(0,1)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(1,1) - spi_2.rec_moter_q(1,1)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(2,1) - spi_2.rec_moter_q(2,1)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(0,2) - spi_2.rec_moter_q(0,2)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(1,2) - spi_2.rec_moter_q(1,2)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(2,2) - spi_2.rec_moter_q(2,2)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(0,3) - spi_2.rec_moter_q(0,3)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(1,3) - spi_2.rec_moter_q(1,3)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(2,3) - spi_2.rec_moter_q(2,3)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(0,4) - spi_2.rec_moter_q(0,4)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(1,4) - spi_2.rec_moter_q(1,4)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(2,4) - spi_2.rec_moter_q(2,4)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(0,5) - spi_2.rec_moter_q(0,5)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(1,5) - spi_2.rec_moter_q(1,5)) >= v_p * radd ||
        fabs(spi_2.rec_moter_q_last(2,5) - spi_2.rec_moter_q(2,5)) >= v_p * radd        
        ){
            data_out_limt(0)++;
            printf("--------------- diff_val_flag: act_q_last - act_q --------------: %f \n", data_out_limt(0));
        }
        else if( 
        fabs(spi_2.rec_moter_v(0,0)) >= 12 ||
        fabs(spi_2.rec_moter_v(1,0)) >= 12 ||
        fabs(spi_2.rec_moter_v(2,0)) >= 12 ||
        fabs(spi_2.rec_moter_v(0,1)) >= 12 ||
        fabs(spi_2.rec_moter_v(1,1)) >= 12 ||
        fabs(spi_2.rec_moter_v(2,1)) >= 12 ||
        fabs(spi_2.rec_moter_v(0,2)) >= 12 ||
        fabs(spi_2.rec_moter_v(1,2)) >= 12 ||
        fabs(spi_2.rec_moter_v(2,2)) >= 12 ||
        fabs(spi_2.rec_moter_v(0,3)) >= 12||
        fabs(spi_2.rec_moter_v(1,3)) >= 12||
        fabs(spi_2.rec_moter_v(2,3)) >= 12||
        fabs(spi_2.rec_moter_v(0,4)) >= 12||
        fabs(spi_2.rec_moter_v(1,4)) >= 12||
        fabs(spi_2.rec_moter_v(2,4)) >= 12||
        fabs(spi_2.rec_moter_v(0,5)) >= 12||
        fabs(spi_2.rec_moter_v(1,5)) >= 12||
        fabs(spi_2.rec_moter_v(2,5)) >= 12       
        ){
            data_out_limt(1)++;
            printf("--------------- vel_val_flag: vel>9 --------------: %f \n", data_out_limt(1));
        }
        else
        {
            MTX_SPICMD.lock();
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
            MTX_SPICMD.unlock();
            // printf("\n ------------ sending ------------ \n");
        }
    }
    #endif
}

void IOSDK::recvState(LowlevelState *state){
    
    // std::cout << "spi_2.rec_moter_q:\n" << spi_2.rec_moter_q *180/3.14 << std::endl;

    #if USE_A_REAL_HEXAPOD == true
        MTX_SPIREC.lock();
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
                    std::cout<<"\n warning : 11rec_moter_q_erroCount:  \n"<< spi_2.rec_moter_q_erroCount <<std::endl;
                    std::cout<<"\n warning : rec_moter_q:  \n"<< spi_2.rec_moter_q <<std::endl;
                    std::cout<<"\n warning : rec_moter_q_last:  \n"<< spi_2.rec_moter_q_last <<"\n" <<std::endl;

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
        MTX_SPIREC.unlock();

        // q校正
        state->motorState[0].q = -state->motorState[0].q;
        state->motorState[2].q = -state->motorState[2].q;

        state->motorState[3].q = -state->motorState[3].q;
        state->motorState[5].q = -state->motorState[5].q;

        state->motorState[6].q = -state->motorState[6].q;
        state->motorState[7].q = -state->motorState[7].q;
        state->motorState[8].q = -state->motorState[8].q;

        state->motorState[9].q = -state->motorState[9].q;
        state->motorState[10].q = -state->motorState[10].q;
        state->motorState[11].q = -state->motorState[11].q;

        state->motorState[12].q = -state->motorState[12].q;
        state->motorState[14].q = -state->motorState[14].q;

        state->motorState[15].q = -state->motorState[15].q;
        state->motorState[17].q = -state->motorState[17].q;

        // t校正
        state->motorState[0].tauEst = -state->motorState[0].tauEst;
        state->motorState[2].tauEst = -state->motorState[2].tauEst;

        state->motorState[3].tauEst = -state->motorState[3].tauEst;
        state->motorState[5].tauEst = -state->motorState[5].tauEst;

        state->motorState[6].tauEst = -state->motorState[6].tauEst;
        state->motorState[7].tauEst = -state->motorState[7].tauEst;
        state->motorState[8].tauEst = -state->motorState[8].tauEst;

        state->motorState[9].tauEst = -state->motorState[9].tauEst;
        state->motorState[10].tauEst = -state->motorState[10].tauEst;
        state->motorState[11].tauEst = -state->motorState[11].tauEst;

        state->motorState[12].tauEst = -state->motorState[12].tauEst;
        state->motorState[14].tauEst = -state->motorState[14].tauEst;

        state->motorState[15].tauEst = -state->motorState[15].tauEst;
        state->motorState[17].tauEst = -state->motorState[17].tauEst;

        // dq校正
        state->motorState[0].dq = -state->motorState[0].dq;
        state->motorState[2].dq = -state->motorState[2].dq;

        state->motorState[3].dq = -state->motorState[3].dq;
        state->motorState[5].dq = -state->motorState[5].dq;

        state->motorState[6].dq = -state->motorState[6].dq;
        state->motorState[7].dq = -state->motorState[7].dq;
        state->motorState[8].dq = -state->motorState[8].dq;

        state->motorState[9].dq = -state->motorState[9].dq;
        state->motorState[10].dq = -state->motorState[10].dq;
        state->motorState[11].dq = -state->motorState[11].dq;

        state->motorState[12].dq = -state->motorState[12].dq;
        state->motorState[14].dq = -state->motorState[14].dq;
        
        state->motorState[15].dq = -state->motorState[15].dq;
        state->motorState[17].dq = -state->motorState[17].dq;

        /* ----------------里面是固定模板，不可动---------------*/
        // q校正
        state->motorState[2].q = -state->motorState[2].q;
        state->motorState[7].q = -state->motorState[7].q;
        state->motorState[13].q = -state->motorState[13].q;

        state->motorState[4].q = -state->motorState[4].q;
        state->motorState[11].q = -state->motorState[11].q;
        state->motorState[17].q = -state->motorState[17].q;

        state->motorState[0].q = -state->motorState[0].q;
        state->motorState[6].q = -state->motorState[6].q;
        state->motorState[12].q = -state->motorState[12].q;

        // t校正
        state->motorState[2].tauEst = -state->motorState[2].tauEst;
        state->motorState[7].tauEst = -state->motorState[7].tauEst;
        state->motorState[13].tauEst = -state->motorState[13].tauEst;

        state->motorState[4].tauEst = -state->motorState[4].tauEst;
        state->motorState[11].tauEst = -state->motorState[11].tauEst;
        state->motorState[17].tauEst = -state->motorState[17].tauEst;

        state->motorState[0].tauEst = -state->motorState[0].tauEst;
        state->motorState[6].tauEst = -state->motorState[6].tauEst;
        state->motorState[12].tauEst = -state->motorState[12].tauEst;

        // dq校正
        state->motorState[2].dq = -state->motorState[2].dq;
        state->motorState[7].dq = -state->motorState[7].dq;
        state->motorState[13].dq = -state->motorState[13].dq;

        state->motorState[4].dq = -state->motorState[4].dq;
        state->motorState[11].dq = -state->motorState[11].dq;
        state->motorState[17].dq = -state->motorState[17].dq;

        state->motorState[0].dq = -state->motorState[0].dq;
        state->motorState[6].dq = -state->motorState[6].dq;
        state->motorState[12].dq = -state->motorState[12].dq;
        /* ----------------里面是固定模板，不可动---------------*/

        std::lock_guard<std::mutex> lock(MTX_IMU); //lcc 自动加锁
        // MTX_IMU.lock();
        for(int i(0); i < 3; ++i){
            state->imu.accelerometer[i] = hipnuc_raw.hi91.acc[i]*GRAVITY;
            state->imu.gyroscope[i] = hipnuc_raw.hi91.gyr[i];
        }
        state->imu.quaternion[0] = hipnuc_raw.hi91.quat[0];
        state->imu.quaternion[1] = hipnuc_raw.hi91.quat[1];
        state->imu.quaternion[2] = hipnuc_raw.hi91.quat[2];
        state->imu.quaternion[3] = hipnuc_raw.hi91.quat[3];
    #endif

}
