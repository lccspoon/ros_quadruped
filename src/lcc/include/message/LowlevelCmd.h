 
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "common/enumClass.h"
#include "interface/KeyBoard.h"

struct MotorCmd{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowlevelCmd{
    MotorCmd motorCmd[NUM_DOF_W];
    // MotorCmd motorCmd[12];

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(Vec18 q){
        for(int i(0); i<18; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3 qi){
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }

    void setQd(Vec12 qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(Vec18 qd){
        for(int i(0); i<18; ++i){
            motorCmd[i].dq = qd(i);
        }
    }

    void setQd(int legID, Vec3 qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }

    void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-50, 50)){
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }

    void setTau(Vec18 tau, Vec2 torqueLimit = Vec2(-25, 25)){
        for(int i(0); i<18; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }

    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<NUM_LEG_W; ++i){
            setZeroDq(i);
        }
    }

    void setLegGain(int legID, float kp, float kd){ //lcc 20240809
        motorCmd[legID*3+0].Kp = kp;
        motorCmd[legID*3+0].Kd = kd;
        motorCmd[legID*3+1].Kp = kp;
        motorCmd[legID*3+1].Kd = kd;
        motorCmd[legID*3+2].Kp = kp;
        motorCmd[legID*3+2].Kd = kd;
    }
    void setAllLegGain(float kp, float kd){ //lcc 20240809
        for (int i = 0; i < 6; i++){
            setLegGain(i, kp, kd);
        }
    }

    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setSimStanceGain(int legID){
        // motorCmd[legID*3+0].mode = 10;
        // motorCmd[legID*3+0].Kp = 180;
        // motorCmd[legID*3+0].Kd = 8;
        // motorCmd[legID*3+1].mode = 10;
        // motorCmd[legID*3+1].Kp = 180;
        // motorCmd[legID*3+1].Kd = 8;
        // motorCmd[legID*3+2].mode = 10;
        // motorCmd[legID*3+2].Kp = 300;
        // motorCmd[legID*3+2].Kd = 15;
    }
    void setRealStanceGain(int legID){
        // motorCmd[legID*3+0].mode = 10;
        // motorCmd[legID*3+0].Kp = 60;
        // motorCmd[legID*3+0].Kd = 5;
        // motorCmd[legID*3+1].mode = 10;
        // motorCmd[legID*3+1].Kp = 40;
        // motorCmd[legID*3+1].Kd = 4;
        // motorCmd[legID*3+2].mode = 10;
        // motorCmd[legID*3+2].Kp = 80;
        // motorCmd[legID*3+2].Kd = 7;
    }
    void setZeroGain(int legID){
        // motorCmd[legID*3+0].mode = 10;
        // motorCmd[legID*3+0].Kp = 0;
        // motorCmd[legID*3+0].Kd = 0;
        // motorCmd[legID*3+1].mode = 10;
        // motorCmd[legID*3+1].Kp = 0;
        // motorCmd[legID*3+1].Kd = 0;
        // motorCmd[legID*3+2].mode = 10;
        // motorCmd[legID*3+2].Kp = 0;
        // motorCmd[legID*3+2].Kd = 0;
    }
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }
    void setStableGain(int legID){
        // motorCmd[legID*3+0].mode = 10;
        // motorCmd[legID*3+0].Kp = 0.8;
        // motorCmd[legID*3+0].Kd = 0.8;
        // motorCmd[legID*3+1].mode = 10;
        // motorCmd[legID*3+1].Kp = 0.8;
        // motorCmd[legID*3+1].Kd = 0.8;
        // motorCmd[legID*3+2].mode = 10;
        // motorCmd[legID*3+2].Kp = 0.8;
        // motorCmd[legID*3+2].Kd = 0.8;
    }
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }
    void setSwingGain(int legID){
        // motorCmd[legID*3+0].mode = 10;
        // motorCmd[legID*3+0].Kp = 3;
        // motorCmd[legID*3+0].Kd = 2;
        // motorCmd[legID*3+1].mode = 10;
        // motorCmd[legID*3+1].Kp = 3;
        // motorCmd[legID*3+1].Kd = 2;
        // motorCmd[legID*3+2].mode = 10;
        // motorCmd[legID*3+2].Kp = 3;
        // motorCmd[legID*3+2].Kd = 2;
    }
};

#endif  //LOWLEVELCMD_H