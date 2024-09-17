 
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

class QuadrupedRobot{
public:
    QuadrupedRobot(){};
    ~QuadrupedRobot(){}

    Vec3 getX(LowlevelState &state);
    Vec34 getVecXP(LowlevelState &state);

    // Inverse Kinematics(Body/Hip Frame)
    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

    Vec34 calcForceByTauEst(const Vec12 &q, const Vec34 feetForce);//lcc

    // Forward Kinematics
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);
    Vec3 getFootVelocity(LowlevelState &state, int id);
    Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);
    Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);

    Mat3 getJaco(LowlevelState &state, int legID);
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}

    double getRobMass(){return _mass;}
    Vec3 getPcb(){return _pcb;}
    Mat3 getRobInertial(){return _Ib;}
    QuadrupedLeg* _Legs[4]; //lcc 20240413   //不同的机器人腿的结构参赛不同，需要具体的在机器人中定义
    Vec34 _feetPosNormalStand;//lcc 20240416 //需要具体的在机器人中定义

protected:
    // QuadrupedLeg* _Legs[4];//lcc 20240413
    Vec2 _robVelLimitX;//需要具体的在机器人中定义
    Vec2 _robVelLimitY;//需要具体的在机器人中定义
    Vec2 _robVelLimitYaw;//需要具体的在机器人中定义
    // Vec34 _feetPosNormalStand;//lcc 20240416
    double _mass;//需要具体的在机器人中定义
    Vec3 _pcb;//需要具体的在机器人中定义
    Mat3 _Ib;//需要具体的在机器人中定义
};

class A1Robot : public QuadrupedRobot{
public:
    A1Robot();
    ~A1Robot(){}
};

class Go1Robot : public QuadrupedRobot{
public:
    Go1Robot();
    ~Go1Robot(){};
};

#endif  // UNITREEROBOT_H