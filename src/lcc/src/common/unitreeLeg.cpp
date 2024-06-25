 
#include "common/unitreeLeg.h"
#include <iostream>


/************************/
/*******QuadrupedLeg*****/
/************************/
QuadrupedLeg::QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                           float kneeLinkLength, Vec3 pHip2B)
            :_abadLinkLength(abadLinkLength), 
             _hipLinkLength(hipLinkLength), 
             _kneeLinkLength(kneeLinkLength), 
             _pHip2B(pHip2B){

    //lcc20240613
    if (legID == 0 || legID == 1 )  
        _sideSign = 1;
    else if (legID == 2 || legID == 3 )
        _sideSign = 0;
    else if (legID == 4 || legID == 5 )
        _sideSign = -1;
    else{
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1);
    }

    if (legID == 0 || legID == 2 || legID == 4)  //lcc: right side
        _sideSign_2 = 1;
    else if (legID == 1 || legID == 3 || legID == 5)
        _sideSign_2 = -1;
    else{
        std::cout << "Leg ID incorrect!" << std::endl;
        exit(-1);
    }

    //ori
    // if (legID == 0 || legID == 2 || legID == 4)  //lcc: right side
    //     _sideSign = 1;
    // else if (legID == 1 || legID == 3 || legID == 5)
    //     _sideSign = -1;
    // else{
    //     std::cout << "Leg ID incorrect!" << std::endl;
    //     exit(-1);
    // }
}

// Forward Kinematics
Vec3 QuadrupedLeg::calcPEe2H(Vec3 q){
    //lcc20240613
    //ori
    float l1 = _sideSign * _abadLinkLength;
    float l2 = -1 * _hipLinkLength;
    float l3 = -_kneeLinkLength;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    Vec3 pEe2H;

    //lcc 20240613
    //为了不改变原有的运动学，直接将四足的运动学套用到六足结构上：
    // （1）原来 四足x方向， 成为了， 六足的z方向
    // （2）原来 四足y方向， 成为了， 六足的x方向
    // （3）原来 四足z方向， 成为了， 六足的y方向
    pEe2H(2) = (l3 * s23 + l2 * s2);// x -> z
    pEe2H(0) = (-l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1);// y -> x
    pEe2H(1) =  _sideSign_2*(l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2);// z -> y
    // pEe2H(1) =  1*(l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2);// z -> y
    // pEe2H(1) =  1*(l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2);// z -> y
    //ori
    // pEe2H(0) = l3 * s23 + l2 * s2;
    // pEe2H(1) = -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1;
    // pEe2H(2) =  l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2;

    return pEe2H;
}

// Forward Kinematics
Vec3 QuadrupedLeg::calcPEe2B(Vec3 q){
    return _pHip2B + calcPEe2H(q);
}

// Derivative Forward Kinematics
Vec3 QuadrupedLeg::calcVEe(Vec3 q, Vec3 qd){
    return calcJaco(q) * qd;
}

// Inverse Kinematics
Vec3 QuadrupedLeg::calcQ(Vec3 pEe, FrameType frame){
    Vec3 pEe2H;
    if(frame == FrameType::HIP)
        pEe2H = pEe;
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    float q1, q2, q3;
    Vec3 qResult;
    float px, py, pz;
    float b2y, b3z, b4z, a, b, c;

    // px = pEe2H(0);
    // py = pEe2H(1);
    // pz = pEe2H(2);

    px = -pEe2H(2);
    // py = -_sideSign_2*pEe2H(1);
    py = pEe2H(0);
    pz = pEe2H(1);

    b2y = _abadLinkLength * _sideSign;
    b3z = -_hipLinkLength;
    b4z = -_kneeLinkLength;
    a = _abadLinkLength* _sideSign;
    // a = _abadLinkLength;
    c = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2)); // whole length
    b = sqrt(pow(c, 2) - pow(a, 2)); // distance between shoulder and footpoint

    q1 = q1_ik(py, pz, b2y);
    q3 = q3_ik(b3z, b4z, b);
    q2 = q2_ik(q1, q3, px, py, pz, b3z, b4z);

    //lcc 20240613
    //为了不改变原有的运动学，直接将四足的运动学套用到六足结构上：
    // （1）原来 四足x方向， 成为了， 六足的z方向
    // （2）原来 四足y方向， 成为了， 六足的x方向
    // （3）原来 四足z方向， 成为了， 六足的y方向
    // pEe2H(2) = (l3 * s23 + l2 * s2);// x -> z
    // pEe2H(0) = (-l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1);// y -> x
    // pEe2H(1) =  -1*(l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2);// z -> y
    qResult(0) = q1;
    // qResult(1) = q2+3.1415926;
    qResult(1) = q2;
    qResult(2) = q3;
    //ori
    // qResult(0) = q1;
    // qResult(1) = q2;
    // qResult(2) = q3;

    return qResult;
}

Eigen::Vector3d QuadrupedLeg::invKinematic(int leg_number, Vec3 pEe, FrameType frame)
{  
    Vec3 pEe2H;
    if(frame == FrameType::HIP)
        pEe2H = pEe;
    else if(frame == FrameType::BODY)
        pEe2H = pEe - _pHip2B;
    else{
        std::cout << "[ERROR] The frame of QuadrupedLeg::calcQ can only be HIP or BODY!" << std::endl;
        exit(-1);
    }

    double LEG_DH_PARAM1;
    if ( leg_number == 2 || leg_number == 3)
    {
         LEG_DH_PARAM1 = 0;
    }
    else if ( leg_number == 4 || leg_number == 5)
    {
         LEG_DH_PARAM1 = -_abadLinkLength;
    }
    else
    {
         LEG_DH_PARAM1 = _abadLinkLength;
    }
    
    double LEG_DH_PARAM2 = _hipLinkLength;
    double LEG_DH_PARAM3 = _kneeLinkLength;

    double invAngleTemp[3], Position[3];
    Position[0]=0.07725;Position[1]=0.12;Position[2]=-0.155;
    // Position[0]=0.07725;Position[1]=0.0;Position[2]=0.205 + 0.216;
    Position[0]=0.07725;Position[1]=0.205 + 0.216;Position[2]=0;
    for (int i = 0; i < 3; i++){
        Position[i]=pEe2H(i);
        // Position[i]= fabs(pEe(i));
    }
    // if ( leg_number == 0 || leg_number == 2 || leg_number == 4)
    // {
    //     Position[1] = -Position[1];
    // }
    // if ( leg_number == 4 || leg_number == 5)
    // {
    //     Position[0] = -Position[0];
    // }
    // if ( leg_number == 2 || leg_number == 3)
    // {
    //     Position[0] = 0.2188;
    // }

    // std::cout<<"pEe2H "<< leg_number <<" :\n"<< pEe2H.transpose() <<std::endl;
    // std::cout<<"Position "<< leg_number <<" :\n"<< Position[0]<<" "<< Position[1]<<" "<< Position[2]<<"\n"<<std::endl;

    //ori
    // if (legID == 0 || legID == 2 || legID == 4)  //lcc: right side
    //     _sideSign = 1;
    // else if (legID == 1 || legID == 3 || legID == 5)
    //     _sideSign = -1;
    // else{
    //     std::cout << "Leg ID incorrect!" << std::endl;
    //     exit(-1);
    // }

    if(leg_number==1 || leg_number==3 || leg_number==5)  
    {
        invAngleTemp[0]=atan2(Position[0],Position[1])-atan2(LEG_DH_PARAM1,pow((Position[1]*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
        invAngleTemp[1]=acos((Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1+LEG_DH_PARAM2*LEG_DH_PARAM2-LEG_DH_PARAM3*LEG_DH_PARAM3)/(2*LEG_DH_PARAM2*pow((Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5)))
        -atan2(-Position[2],pow((Position[1]*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
        invAngleTemp[2]=acos( (LEG_DH_PARAM2*LEG_DH_PARAM2+LEG_DH_PARAM3*LEG_DH_PARAM3-(Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1))/(2*LEG_DH_PARAM2*LEG_DH_PARAM3) )-3.1416;
    }
    else
    {    // 左边和右边的运动学不一样，主要在于右边y方向要加负号
        int lambda=-1;
        invAngleTemp[0]=atan2(Position[0],lambda*Position[1])-atan2(LEG_DH_PARAM1,pow((lambda*Position[1]*lambda*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
        invAngleTemp[1]=acos((Position[0]*Position[0]+lambda*Position[1]*lambda*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1+LEG_DH_PARAM2*LEG_DH_PARAM2-LEG_DH_PARAM3*LEG_DH_PARAM3)/(2*LEG_DH_PARAM2*pow((Position[0]*Position[0]+Position[1]*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5)))
        -atan2(-Position[2],pow((lambda*Position[1]*lambda*Position[1]+Position[0]*Position[0]-LEG_DH_PARAM1*LEG_DH_PARAM1),0.5));
        invAngleTemp[2]=acos( (LEG_DH_PARAM2*LEG_DH_PARAM2+LEG_DH_PARAM3*LEG_DH_PARAM3-(Position[0]*Position[0]+lambda*Position[1]*lambda*Position[1]+Position[2]*Position[2]-LEG_DH_PARAM1*LEG_DH_PARAM1))/(2*LEG_DH_PARAM2*LEG_DH_PARAM3) )-3.1416;
    }

    if( std::isinf(invAngleTemp[0]) || std::isnan(invAngleTemp[0]))  ;    else
        invAngle[0]=invAngleTemp[0];
    if( std::isinf(invAngleTemp[1]) || std::isnan(invAngleTemp[1]))  ;    else
        invAngle[1]=invAngleTemp[1];
    if( std::isinf(invAngleTemp[2]) || std::isnan(invAngleTemp[2]))  ;    else
        invAngle[2]=invAngleTemp[2];
    // std::cout<<"Des  "<<Position[0]<<" "<<Position[1]<<" "<<Position[2]<<std::endl;
    // std::cout<<"Ang  "<<invAngle[0]*180/3.1416<<" "<<invAngle[1]*180/3.1416<<" "<<invAngle[2]*180/3.1416<<std::endl;

    Eigen::Vector3d invANg;
    for (int i = 0; i < 3; i++)
    {
        invANg(i)=invAngle[i];
    }
    return invANg;
}


// Derivative Inverse Kinematics
Vec3 QuadrupedLeg::calcQd(Vec3 q, Vec3 vEe){
    return calcJaco(q).inverse() * vEe;
}

// Derivative Inverse Kinematics
Vec3 QuadrupedLeg::calcQd(Vec3 pEe, Vec3 vEe, FrameType frame){
    Vec3 q = calcQ(pEe, frame);
    return calcJaco(q).inverse() * vEe;
}

// Inverse Dynamics
Vec3 QuadrupedLeg::calcTau(Vec3 q, Vec3 force){
    return calcJaco(q).transpose() * force;
}

// Jacobian Matrix
Mat3 QuadrupedLeg::calcJaco(Vec3 q){
    Mat3 jaco;

    float l1 = _abadLinkLength * _sideSign;
    float l2 = -_hipLinkLength;
    float l3 = -_kneeLinkLength;

    float s1 = std::sin(q(0));
    float s2 = std::sin(q(1));
    float s3 = std::sin(q(2));

    float c1 = std::cos(q(0));
    float c2 = std::cos(q(1));
    float c3 = std::cos(q(2));

    float c23 = c2 * c3 - s2 * s3;
    float s23 = s2 * c3 + c2 * s3;

    //lcc 20240613
    //为了不改变原有的运动学，直接将四足的运动学套用到六足结构上：
    // （1）原来 四足x方向， 成为了， 六足的z方向
    // （2）原来 四足y方向， 成为了， 六足的x方向
    // （3）原来 四足z方向， 成为了， 六足的y方向
    // pEe2H(2) = (l3 * s23 + l2 * s2);// x -> z
    // pEe2H(0) = (-l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1);// y -> x
    // pEe2H(1) =  -1*(l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2);// z -> y
    jaco(2, 0) = 0;
    jaco(2, 1) = l3 * c23 + l2 * c2;
    jaco(2, 2) = l3 * c23;

    jaco(0, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    jaco(0, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    jaco(0, 2) = l3 * s1 * s23;

    jaco(1, 0) = _sideSign_2*(-l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1);
    jaco(1, 1) = _sideSign_2*(-l3 * c1 * s23 - l2 * c1 * s2);
    jaco(1, 2) = _sideSign_2*(-l3 * c1 * s23);

    // jaco(0, 0) = 0;
    // jaco(0, 1) = l3 * c23 + l2 * c2;
    // jaco(0, 2) = l3 * c23;

    // jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    // jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    // jaco(1, 2) = l3 * s1 * s23;

    // jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    // jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    // jaco(2, 2) = -l3 * c1 * s23;

    //ori
    // jaco(0, 0) = 0;
    // jaco(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
    // jaco(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
    // jaco(0, 1) = l3 * c23 + l2 * c2;
    // jaco(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
    // jaco(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
    // jaco(0, 2) = l3 * c23;
    // jaco(1, 2) = l3 * s1 * s23;
    // jaco(2, 2) = -l3 * c1 * s23;

    return jaco;
}

float QuadrupedLeg::q1_ik(float py, float pz, float l1){
    float q1;
    float L = sqrt(pow(py,2)+pow(pz,2)-pow(l1,2));
    q1 = atan2(pz*l1+py*L, py*l1-pz*L);
    return q1;
}

float QuadrupedLeg::q3_ik(float b3z, float b4z, float b){
    float q3, temp;
    temp = (pow(b3z, 2) + pow(b4z, 2) - pow(b, 2))/(2*fabs(b3z*b4z));
    if(temp>1) temp = 1;
    if(temp<-1) temp = -1;
    q3 = acos(temp);
    q3 = -(M_PI - q3); //0~180
    return q3;
}

float QuadrupedLeg::q2_ik(float q1, float q3, float px, float py, float pz, float b3z, float b4z){
    float q2, a1, a2, m1, m2;
    
    a1 = py*sin(q1) - pz*cos(q1);
    a2 = px;
    m1 = b4z*sin(q3);
    m2 = b3z + b4z*cos(q3);
    q2 = atan2(m1*a1+m2*a2, m1*a2-m2*a1);
    return q2;
}


