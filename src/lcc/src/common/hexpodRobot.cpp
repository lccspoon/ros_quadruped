#include "common/hexpodRobot.h"
#include <iostream>

Vec3 HexapodRobot::getX(LowlevelState &state){
    return getFootPosition(state, 0, FrameType::BODY);
}

Vec36 HexapodRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);
    Vec36 vecXP, qLegs;
    qLegs = state.getQ_Hex();

    for(int i(0); i < 6; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}
// Inverse Kinematics
Vec18 HexapodRobot::getQ(const Vec36 &vecP, FrameType frame){
    Vec18 q;
    // for(int i(0); i < 6; ++i){
    //     q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    // }
    for(int i(0); i < 6; ++i){
        q.segment(3*i, 3) = _Legs[i]->invKinematic(i, vecP.col(i), frame);
    }

    return q;
}

Vec18 HexapodRobot::getQd(const Vec36 &pos, const Vec36 &vel, FrameType frame){
    Vec18 qd;
    for(int i(0); i < 6; ++i){
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

Vec18 HexapodRobot::getTau(const Vec18 &q, const Vec36 feetForce){
    Vec18 tau;
    for(int i(0); i < 6; ++i){
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }

    return tau;
}

Vec36 HexapodRobot::calcForceByTauEst(const Vec18 &q, const Vec36 feetForce)//lcc
{
    Vec36 f;
    for(int i(0); i < 6; ++i){
        f.block<3, 1>(0, i) = ( _Legs[i]->calcJaco( q.segment(3*i, 3) ).transpose() ).inverse() * feetForce.block<3, 1>(0, i);
    }
    return f;
}

// Forward Kinematics
Vec3 HexapodRobot::getFootPosition(LowlevelState &state, int id, FrameType frame){
    Vec36 qLegs= state.getQ_Hex();

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
Vec3 HexapodRobot::getFootVelocity(LowlevelState &state, int id){
    Vec36 qLegs = state.getQ_Hex();
    Vec36 qdLegs= state.getQd_Hex();
    // std::cout << "qLegs:\n" <<qLegs<< std::endl;
    // std::cout << "qdLegs:\n" <<qdLegs<< std::endl;
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

// Forward Kinematics
Vec36 HexapodRobot::getFeet2BPositions(LowlevelState &state, FrameType frame){
    Vec36 feetPos;
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<6; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
        }
        feetPos = state.getRotMat() * feetPos;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        for(int i(0); i<6; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

Vec36 HexapodRobot::getFeet2BVelocities(LowlevelState &state, FrameType frame){
    Vec36 feetVel;
    for(int i(0); i<6; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }

    if(frame == FrameType::GLOBAL){
        // #if USE_A_REAL_HEXAPOD == true
        return state.getRotMat() * feetVel;
        // #else
        // Vec36 feetPos = getFeet2BPositions(state, FrameType::BODY);
        // feetVel += skew(state.getGyro()) * feetPos;
        // return state.getRotMat() * feetVel;
        // #endif
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

Mat3 HexapodRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ_Hex().col(legID));
}

SixLegDogRobot::SixLegDogRobot(){

    #if USE_A_REAL_HEXAPOD == true
    _Legs[0] = new SixLegDogLeg(0, Vec3( 0.155, -0.075, 0));//rf  //这里输入的是 pHip2B 20240524
    _Legs[1] = new SixLegDogLeg(1, Vec3( 0.155,  0.075, 0));//lf
    _Legs[2] = new SixLegDogLeg(2, Vec3(0.0, -0.075, 0));//rm
    _Legs[3] = new SixLegDogLeg(3, Vec3(0.0,  0.075, 0));//lm
    _Legs[4] = new SixLegDogLeg(4, Vec3(-0.155, -0.075, 0));//rb
    _Legs[5] = new SixLegDogLeg(5, Vec3(-0.155,  0.075, 0));//lb
    //lcc 实际上，这个理想位置就是：body系下 x和y值就是(足端+小腿+大腿)平面,即LX和LY+L1。z方向的值还不知道怎么确定
    //hip下的投影
    // _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
    //                        -0.075 - 0.12,  0.075 + 0.12, -0.075 - 0.12,  0.075 + 0.12, -0.075 - 0.12,  0.075 + 0.12,
    //                        -0.21, -0.21, -0.21, -0.21, -0.21, -0.21;

    _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076 -0.01, -0.155 - 0.076-0.01, 
                           -0.075 - 0.17,  0.075 + 0.17, -0.075 - 0.17 - 0.02,  0.075 + 0.17 + 0.02, -0.075 - 0.17,  0.075 + 0.17,
                           -0.23, -0.23, -0.23, -0.23, -0.23, -0.23;

    // _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
    //                        -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13,
    //                        -0.24, -0.24, -0.24, -0.24, -0.24, -0.24;

    // _feetPosNormalStand <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
    //                        -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13, -0.075 - 0.13,  0.075 + 0.13,
    //                        -0.27, -0.27, -0.27, -0.27, -0.27, -0.27;

    _feetPosNormalSquat <<  0.155 + 0.076,  0.155 + 0.076, 0.0, 0.0, -0.155 - 0.076, -0.155 - 0.076, 
                           -0.075 - 0.34,  0.075 + 0.34, -0.075 - 0.34,  0.075 + 0.34, -0.075 - 0.34,  0.075 + 0.34,
                           -0.015, -0.015, -0.015, -0.015, -0.015, -0.015;

    _robVelLimitX << -0.1, 0.1;
    _robVelLimitY << -0.2, 0.2; // 力位混合，最快 vy=0.3m/s
    _robVelLimitYaw << -0.2, 0.2;

    _mass =15;
    // _mass =20;
    _pcb << 0.0, 0.0, 0.07;

    // ixx="0.026264"
    // ixy="0"
    // ixz="0"
    // iyy="0.069583"
    // iyz="0"
    // izz="0.090006" />

    // _Ib = Vec3(0.26264, 0.69583, 0.90006).asDiagonal(); //原始的转动惯量
    _Ib = Vec3(1, 1, 1).asDiagonal();//lcc 20240611: 修改后的转的惯量 -> 我发现换上这个以后，往前走也不会沉头了，效果好了很多
    #else
    _Legs[0] = new SixLegDogLeg(0, Vec3( 0.14185, -0.0655, 0));//rf  //这里输入的是 pHip2B 20240524
    _Legs[1] = new SixLegDogLeg(1, Vec3( 0.14185,  0.0655, 0));//lf
    _Legs[2] = new SixLegDogLeg(2, Vec3(0.0, -0.0655, 0));//rm
    _Legs[3] = new SixLegDogLeg(3, Vec3(0.0,  0.0655, 0));//lm
    _Legs[4] = new SixLegDogLeg(4, Vec3(-0.14185, -0.0655, 0));//rb
    _Legs[5] = new SixLegDogLeg(5, Vec3(-0.14185,  0.0655, 0));//lb
    //lcc 实际上，这个理想位置就是：body系下 x和y值就是(足端+小腿+大腿)平面,即LX和LY+L1。z方向的值还不知道怎么确定
    //hip下的投影
    _feetPosNormalStand <<  0.14185 + 0.07725,  0.14185 + 0.07725, 0.0, 0.0, -0.14185 - 0.07725, -0.14185 - 0.07725, 
                           -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13,
                           -0.24, -0.24, -0.24, -0.24, -0.24, -0.24;

    // _feetPosNormalStand <<  0.14185 + 0.07725+0.05,  0.14185 + 0.07725+0.05, 0.0+0.05, 0.0+0.05, -0.14185 - 0.07725+0.05, -0.14185 - 0.07725+0.05, 
    //                        -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13, -0.0655 - 0.13,  0.0655 + 0.13,
    //                        -0.24, -0.24, -0.24, -0.24, -0.24, -0.24;

    _feetPosNormalSquat <<  0.14185 + 0.07725,  0.14185 + 0.07725, 0.0, 0.0, -0.14185 - 0.07725, -0.14185 - 0.07725, 
                           -0.0655 - 0.34,  0.0655 + 0.34, -0.0655 - 0.34,  0.0655 + 0.34, -0.0655 - 0.34,  0.0655 + 0.34,
                           -0.025, -0.025, -0.025, -0.025, -0.025, -0.025;

    _robVelLimitX << -0.1, 0.1;
    _robVelLimitY << -0.2, 0.2; // 力位混合，最快 vy=0.3m/s
    _robVelLimitYaw << -0.2, 0.2;

    // _mass =16;
    _mass =16.8;
    // _mass =20;
    _pcb << 0.0, 0.0, 0.0;

    // ixx="0.026264"
    // ixy="0"
    // ixz="0"
    // iyy="0.069583"
    // iyz="0"
    // izz="0.090006" />

    // _Ib = Vec3(0.26264, 0.69583, 0.90006).asDiagonal(); //原始的转动惯量
    _Ib = Vec3(1, 1, 1).asDiagonal();//lcc 20240611: 修改后的转的惯量 -> 我发现换上这个以后，往前走也不会沉头了，效果好了很多
    #endif
}

