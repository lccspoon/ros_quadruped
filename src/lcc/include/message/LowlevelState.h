 
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"
#include "interface/KeyBoard.h"

struct MotorState
{
	unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];    // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];

    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMat getRotMat(){
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc(){
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro(){
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat(){
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    LowlevelState(){
        realRotMat.setZero();
        offsRotMat.setZero();
        retuRotMat.setZero();
        defaRotMat.setZero();
        defaRotMat.setIdentity();

        retuYaw = 0;
        offsYaw = 0;
    }

    IMU imu;
    MotorState motorState[NUM_DOF_W];
    // MotorState motorState[18];
    // MotorState motorState[12];
    UserCommand userCmd;
    UserValue userValue;
    UserFunctionMode userFunctionMode;// lcc 20250601
    // UserFunctionMode *userFunctionMode_p; // lcc 20250601

    // #if IS_THIS_A_HEXAPOD
        Vec36 getQ_Hex(){
            Vec36 qLegs;
            for(int i(0); i < NUM_LEG_W; ++i){
            // for(int i(0); i < 4; ++i){
                qLegs.col(i)(0) = motorState[3*i    ].q;
                qLegs.col(i)(1) = motorState[3*i + 1].q;
                qLegs.col(i)(2) = motorState[3*i + 2].q;
            }
            return qLegs;
        }
        Vec36 getQd_Hex(){
            Vec36 qdLegs;
            for(int i(0); i < NUM_LEG_W; ++i){
            // for(int i(0); i < 4; ++i){
                qdLegs.col(i)(0) = motorState[3*i    ].dq;
                qdLegs.col(i)(1) = motorState[3*i + 1].dq;
                qdLegs.col(i)(2) = motorState[3*i + 2].dq;
            }
            return qdLegs;
        }
        Vec36 getTau_Hex(){
            Vec36 tauLegs;
            for(int i(0); i < NUM_LEG_W; ++i){
            // for(int i(0); i < 4; ++i){
                tauLegs.col(i)(0) = motorState[3*i    ].tauEst;
                tauLegs.col(i)(1) = motorState[3*i + 1].tauEst;
                tauLegs.col(i)(2) = motorState[3*i + 2].tauEst;
            }
            return tauLegs;
        }
    // #else
        Vec34 getQ(){
            Vec34 qLegs;
            for(int i(0); i < NUM_LEG_W; ++i){
            // for(int i(0); i < 4; ++i){
                qLegs.col(i)(0) = motorState[3*i    ].q;
                qLegs.col(i)(1) = motorState[3*i + 1].q;
                qLegs.col(i)(2) = motorState[3*i + 2].q;
            }
            return qLegs;
        }
        Vec34 getQd(){
            Vec34 qdLegs;
            for(int i(0); i < NUM_LEG_W; ++i){
            // for(int i(0); i < 4; ++i){
                qdLegs.col(i)(0) = motorState[3*i    ].dq;
                qdLegs.col(i)(1) = motorState[3*i + 1].dq;
                qdLegs.col(i)(2) = motorState[3*i + 2].dq;
            }
            return qdLegs;
        }
        Vec34 getTau(){
            Vec34 tauLegs;
            for(int i(0); i < NUM_LEG_W; ++i){
            // for(int i(0); i < 4; ++i){
                tauLegs.col(i)(0) = motorState[3*i    ].tauEst;
                tauLegs.col(i)(1) = motorState[3*i + 1].tauEst;
                tauLegs.col(i)(2) = motorState[3*i + 2].tauEst;
            }
            return tauLegs;
        }
    // #endif

    Eigen::Matrix<double, 3, 3> realRotMat;
    Eigen::Matrix<double, 3, 3> offsRotMat;
    Eigen::Matrix<double, 3, 3> retuRotMat;
    Eigen::Matrix<double, 3, 3> defaRotMat;
    
    RotMat getRotMat(){

        retuRotMat = imu.getRotMat();
        // std::cout<<" asasas-state_reset: \n"<< userFunctionMode.state_reset <<std::endl;

        if( userFunctionMode.state_reset == true){
            offsRotMat = defaRotMat - retuRotMat;
        }

        // std::cout<<" retuRotMat: \n"<< retuRotMat <<std::endl;
        // std::cout<<" retuRotMat + offsRotMat: \n"<< retuRotMat + offsRotMat <<std::endl;

        return retuRotMat + offsRotMat;
    }

    Vec3 getAcc(){
        return imu.getAcc();
    }

    Vec3 getGyro(){
        return imu.getGyro();
    }

    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    double retuYaw;
    double offsYaw;

    double getYaw(){
        retuYaw = rotMatToRPY(getRotMat())(2);
        if( userFunctionMode.state_reset == true){
            offsYaw = 0 - retuYaw;
        }

        // std::cout<<" retuYaw + offsYaw: \n"<< retuYaw + offsYaw <<std::endl;

        return retuYaw + offsYaw;
    }

    double getDYaw(){
        return getGyroGlobal()(2);
    }

    // void setQ(Vec12 q){
    //     for(int i(0); i<12; ++i){
    //         motorState[i].q = q(i);
    //     }
    // }
};

#endif  //LOWLEVELSTATE_HPP