#ifndef HEXAPOD_ROBOT_H
#define HEXAPOD_ROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

class HexapodRobot{
 private:
    double invAngle[3];
public:
    HexapodRobot(){};
    ~HexapodRobot(){}

    Vec3 getX(LowlevelState &state);
    Vec36 getVecXP(LowlevelState &state);

    // Inverse Kinematics(Body/Hip Frame)
    Vec18 getQ(const Vec36 &feetPosition, FrameType frame);
    Vec18 getQd(const Vec36 &feetPosition, const Vec36 &feetVelocity, FrameType frame);
    Vec18 getTau(const Vec18 &q, const Vec36 feetForce);

    Vec36 calcForceByTauEst(const Vec18 &q, const Vec36 feetForce);//lcc

    // Forward Kinematics
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);
    Vec3 getFootVelocity(LowlevelState &state, int id);
    Vec36 getFeet2BPositions(LowlevelState &state, FrameType frame);
    Vec36 getFeet2BVelocities(LowlevelState &state, FrameType frame);

    Mat3 getJaco(LowlevelState &state, int legID);
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec36 getFeetPosIdeal(){return _feetPosNormalStand;}

    double getRobMass(){return _mass;}
    Vec3 getPcb(){return _pcb;}
    Mat3 getRobInertial(){return _Ib;}
    QuadrupedLeg* _Legs[6]; //lcc 20240413
    Vec36 _feetPosNormalStand;//lcc 20240416
    Vec36 _feetPosNormalSquat;//lcc 20240808

protected:
    // QuadrupedLeg* _Legs[6];//lcc 20240413
    Vec2 _robVelLimitX;
    Vec2 _robVelLimitY;
    Vec2 _robVelLimitYaw;
    // Vec36 _feetPosNormalStand;//lcc 20240416
    double _mass;
    Vec3 _pcb;
    Mat3 _Ib;
};

class SixLegDogRobot : public HexapodRobot{
public:
    SixLegDogRobot();
    ~SixLegDogRobot(){}
};

#endif 