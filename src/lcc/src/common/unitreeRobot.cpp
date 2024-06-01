/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/unitreeRobot.h"
#include <iostream>

Vec3 QuadrupedRobot::getX(LowlevelState &state){
    return getFootPosition(state, 0, FrameType::BODY);
}

Vec34 QuadrupedRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);
    Vec34 vecXP, qLegs;
    qLegs = state.getQ();

    for(int i(0); i < 4; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}
// Inverse Kinematics
Vec12 QuadrupedRobot::getQ(const Vec34 &vecP, FrameType frame){
    Vec12 q;
    for(int i(0); i < 4; ++i){
        q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    }
    return q;
}

Vec12 QuadrupedRobot::getQd(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec12 qd;
    for(int i(0); i < 4; ++i){
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

Vec12 QuadrupedRobot::getTau(const Vec12 &q, const Vec34 feetForce){
    Vec12 tau;
    for(int i(0); i < 4; ++i){
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}

Vec34 QuadrupedRobot::calcForceByTauEst(const Vec12 &q, const Vec34 feetForce)//lcc
{
    Vec34 f;
    for(int i(0); i < 4; ++i){
        f.block<3, 1>(0, i) = ( _Legs[i]->calcJaco( q.segment(3*i, 3) ).transpose() ).inverse() * feetForce.block<3, 1>(0, i);
    }
    return f;
}

// Forward Kinematics
Vec3 QuadrupedRobot::getFootPosition(LowlevelState &state, int id, FrameType frame){
    Vec34 qLegs= state.getQ();

    if(frame == FrameType::BODY){
        return _Legs[id]->calcPEe2B(qLegs.col(id));
    }else if(frame == FrameType::HIP){
        return _Legs[id]->calcPEe2H(qLegs.col(id));
    }else{
        std::cout << "[ERROR] The frame of function: getFootPosition can only be BODY or HIP." << std::endl;
        exit(-1);
    }
}

// Forward derivative Kinematics
Vec3 QuadrupedRobot::getFootVelocity(LowlevelState &state, int id){
    Vec34 qLegs = state.getQ();
    Vec34 qdLegs= state.getQd();
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

// Forward Kinematics
Vec34 QuadrupedRobot::getFeet2BPositions(LowlevelState &state, FrameType frame){
    Vec34 feetPos;
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
        }
        feetPos = state.getRotMat() * feetPos;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

Vec34 QuadrupedRobot::getFeet2BVelocities(LowlevelState &state, FrameType frame){
    Vec34 feetVel;
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }

    if(frame == FrameType::GLOBAL){
        Vec34 feetPos = getFeet2BPositions(state, FrameType::BODY);
        feetVel += skew(state.getGyro()) * feetPos;
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

Mat3 QuadrupedRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

// // 结构参数:{hx, hy, l1, l2, l3}
// #define HX 0.2255 
// #define HY 0.065 
// #define L1 0.06725 
// #define L2 0.261 
// #define L3 0.196 

A1Robot::A1Robot(){
    // _Legs[0] = new A1Leg(0, Vec3( 0.1805, -0.047, 0));//rf
    // _Legs[1] = new A1Leg(1, Vec3( 0.1805,  0.047, 0));//lf
    // _Legs[2] = new A1Leg(2, Vec3(-0.1805, -0.047, 0));//rb
    // _Legs[3] = new A1Leg(3, Vec3(-0.1805,  0.047, 0));//lb
    _Legs[0] = new A1Leg(0, Vec3( 0.225, -0.065, 0));//rf  //这里输入的是 pHip2B 20240524
    _Legs[1] = new A1Leg(1, Vec3( 0.225,  0.065, 0));//lf
    _Legs[2] = new A1Leg(2, Vec3(-0.225, -0.065, 0));//rb
    _Legs[3] = new A1Leg(3, Vec3(-0.225,  0.065, 0));//lb
    //lcc 实际上，这个理想位置就是：body系下 x和y值就是(足端+小腿+大腿)平面,即LX和LY+L1。z方向的值还不知道怎么确定
    _feetPosNormalStand <<  0.225,  0.225, -0.225, -0.225, 
                           -0.14325,  0.14325, -0.14325,  0.14325,
                           -0.3180, -0.3180, -0.3180, -0.3180;

    _robVelLimitX << -0.4, 0.4;
    _robVelLimitY << -0.3, 0.3;
    _robVelLimitYaw << -0.5, 0.5;
    
    //lcc
    // _robVelLimitX << -0.5, 0.5;
    // _robVelLimitY << -0.4, 0.4;
    // _robVelLimitYaw << -0.5, 0.5;

    // #ifdef COMPILE_WITH_REAL_ROBOT
    //     _mass = 12.5;
    //     _pcb << 0.01, 0.0, 0.0;
    //     _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();
    // #endif  // COMPILE_WITH_REAL_ROBOT

    // #ifdef COMPILE_WITH_SIMULATION



        // _mass = 13.4;
        // _pcb << 0.0, 0.0, 0.0;
        // _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();


        _mass =16;
        _pcb << 0.0, 0.0, 0.0;
        _Ib = Vec3(0.132, 0.3475, 0.3775).asDiagonal();



    // #endif  // COMPILE_WITH_SIMULATION
}

Go1Robot::Go1Robot(){
//     _Legs[0] = new Go1Leg(0, Vec3( 0.1881, -0.04675, 0));
//     _Legs[1] = new Go1Leg(1, Vec3( 0.1881,  0.04675, 0));
//     _Legs[2] = new Go1Leg(2, Vec3(-0.1881, -0.04675, 0));
//     _Legs[3] = new Go1Leg(3, Vec3(-0.1881,  0.04675, 0));

//     _feetPosNormalStand <<  0.1881,  0.1881, -0.1881, -0.1881,
//                            -0.1300,  0.1300, -0.1300,  0.1300,
//                            -0.3200, -0.3200, -0.3200, -0.3200;
//     _robVelLimitX << -0.4, 0.4;
//     _robVelLimitY << -0.3, 0.3;
//     _robVelLimitYaw << -0.5, 0.5;

//     //lcc
//     // _robVelLimitX << -0.5, 0.5;
//     // _robVelLimitY << -0.4, 0.4;
//     // _robVelLimitYaw << -0.5, 0.5;


// // #ifdef COMPILE_WITH_REAL_ROBOT
// //     _mass = 10.5;
// //     _pcb << 0.04, 0.0, 0.0;
// //     _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
// // #endif  // COMPILE_WITH_REAL_ROBOT

// // #ifdef COMPILE_WITH_SIMULATION
//     _mass = 12.0;
//     _pcb << 0.0, 0.0, 0.0;
//     _Ib = Vec3(0.0792, 0.2085, 0.2265).asDiagonal();
// // #endif  // COMPILE_WITH_SIMULATION
}