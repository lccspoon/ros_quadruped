 
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

class QuadrupedLeg{
private:
    double invAngle[3];
public:
    QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B);
    ~QuadrupedLeg(){}
    Vec3 calcPEe2H(Vec3 q);
    Vec3 calcPEe2B(Vec3 q);
    Vec3 calcVEe(Vec3 q, Vec3 qd);
    Vec3 calcQ(Vec3 pEe, FrameType frame);
    Vec3 invKinematic(int leg_number, Vec3 pEe, FrameType frame); // lcc 20240616
    Vec3 calcQd(Vec3 q, Vec3 vEe);
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);
    Vec3 calcTau(Vec3 q, Vec3 force);
    Mat3 calcJaco(Vec3 q);
    // Mat3 calcForceByTauEst(Vec3 q);//lcc
    Vec3 getHip2B(){return _pHip2B;}
    const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;//lcc 20240413
protected:
    float q1_ik(float py, float pz, float b2y);
    float q3_ik(float b3z, float b4z, float b);
    float q2_ik(float q1, float q3, float px, 
                float py, float pz, float b3z, float b4z);
    float _sideSign;
    float _sideSign_2;
    // const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;//lcc 20240413
    const Vec3 _pHip2B;
};

class A1Leg : public QuadrupedLeg{
public:
    // 在初始化列表中，首先调用了基类 QuadrupedLeg 的构造函数，
    // 传递了参数 legID、0.0838、0.2、0.2 和 pHip2B。
    // 这样做会调用基类 QuadrupedLeg 的构造函数，并传递这些参数来初始化基类的成员变量 
    // _abadLinkLength、_hipLinkLength、_kneeLinkLength 和 _pHip2B。
    A1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.07725, 0.216, 0.205, pHip2B){}//lcc 20240524
        // QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B){}
    ~A1Leg(){}
};

class Go1Leg : public QuadrupedLeg{
public:
    Go1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B){}
    ~Go1Leg(){}
};

//lcc 20240608
class SixLegDogLeg : public QuadrupedLeg{
public:
    SixLegDogLeg(const int legID, const Vec3 pHip2B):
        #if USE_A_REAL_HEXAPOD == true
        QuadrupedLeg(legID, 0.076, 0.2115, 0.215, pHip2B){}//lcc 20240828
        #else
        QuadrupedLeg(legID, 0.07725, 0.216, 0.205, pHip2B){}//lcc 20240524
        #endif
    ~SixLegDogLeg(){}
};


#endif  // UNITREELEG_H