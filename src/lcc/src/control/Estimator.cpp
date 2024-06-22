/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "control/Estimator.h"
#include "common/mathTools.h"
#include "common/enumClass.h"
#include "interface/IOROS.h"
Estimator::Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, 
                     VecInt4 *contact, Vec4 *phase, double dt, Vec18 Qdig,
                     std::string testName)
          :_robModel(robotModel), _lowState(lowState), _contact(contact),
           _phase(phase), _dt(dt), _Qdig(Qdig), _estName(testName){

    _initSystem();
}

Estimator::Estimator(QuadrupedRobot *robotModel, LowlevelState* lowState, 
                     VecInt4 *contact, Vec4 *phase, double dt)
          :_robModel(robotModel), _lowState(lowState), _contact(contact), 
           _phase(phase), _dt(dt){

    for(int i(0); i<_Qdig.rows(); ++i){
        if(i < 3){
            _Qdig(i) = 0.0003;
        }
        else if(i < 6){
            _Qdig(i) = 0.0003;
        }
        else{
            _Qdig(i) = 0.01;
        }
    }

    _estName = "current";

    accOffset.setZero();
    postionOffset.setZero();
    // _Qdig(5) = 1; //lcc
    _initSystem();

}

Estimator::~Estimator(){
}

void Estimator::_initSystem(){
    _g << 0, 0, -9.81;
    _largeVariance = 100;

    _xhat.setZero();
    _u.setZero();
    _A.setZero();
    _A.block(0, 0, 3, 3) = I3;
    _A.block(0, 3, 3, 3) = I3 * _dt;
    _A.block(3, 3, 3, 3) = I3;
    _A.block(6, 6, 12, 12) = I12;
    _B.setZero();
    _B.block(3, 0, 3, 3) = I3 * _dt;
    _C.setZero();
    _C.block(0, 0, 3, 3) = -I3;
    _C.block(3, 0, 3, 3) = -I3;
    _C.block(6, 0, 3, 3) = -I3;
    _C.block(9, 0, 3, 3) = -I3;
    _C.block(12, 3, 3, 3) = -I3;
    _C.block(15, 3, 3, 3) = -I3;
    _C.block(18, 3, 3, 3) = -I3;
    _C.block(21, 3, 3, 3) = -I3;
    _C.block(0, 6, 12, 12) = I12;
    _C(24, 8) = 1;
    _C(25, 11) = 1;
    _C(26, 14) = 1;
    _C(27, 17) = 1;
    _P.setIdentity();
    _P = _largeVariance * _P;

    // _RInit <<  0.008 , 0.012 ,-0.000 ,-0.009 , 0.012 , 0.000 , 0.009 ,-0.009 ,-0.000 ,-0.009 ,-0.009 , 0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 ,-0.000 ,-0.001 ,-0.002 , 0.000 ,-0.000 ,-0.003 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
    //            0.012 , 0.019 ,-0.001 ,-0.014 , 0.018 ,-0.000 , 0.014 ,-0.013 ,-0.000 ,-0.014 ,-0.014 , 0.001 ,-0.001 , 0.001 ,-0.001 , 0.000 , 0.000 ,-0.001 ,-0.003 , 0.000 ,-0.001 ,-0.004 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
    //            -0.000, -0.001,  0.001,  0.001, -0.001,  0.000, -0.000,  0.000, -0.000,  0.001,  0.000, -0.000,  0.000, -0.000,  0.000,  0.000, -0.000, -0.000,  0.000, -0.000, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,
    //            -0.009, -0.014,  0.001,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001,  0.000,  0.000,  0.001, -0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
    //            0.012 , 0.018 ,-0.001 ,-0.013 , 0.018 ,-0.000 , 0.013 ,-0.013 ,-0.000 ,-0.013 ,-0.013 , 0.001 ,-0.001 , 0.000 ,-0.001 , 0.000 , 0.001 ,-0.001 ,-0.003 , 0.000 ,-0.001 ,-0.004 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
    //            0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 , 0.001 , 0.000 , 0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000,
    //            0.009 , 0.014 ,-0.000 ,-0.010 , 0.013 , 0.000 , 0.010 ,-0.010 ,-0.000 ,-0.010 ,-0.010 , 0.000 ,-0.001 , 0.000 ,-0.001 , 0.000 ,-0.000 ,-0.001 ,-0.001 , 0.000 ,-0.000 ,-0.003 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
    //            -0.009, -0.013,  0.000,  0.010, -0.013,  0.000, -0.010,  0.009,  0.000,  0.010,  0.010, -0.000,  0.001, -0.000,  0.000, -0.000,  0.000,  0.001,  0.002,  0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
    //            -0.000, -0.000, -0.000,  0.000, -0.000, -0.000, -0.000,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000,  0.000, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,
    //            -0.009, -0.014,  0.001,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001,  0.000,  0.000, -0.000, -0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
    //            -0.009, -0.014,  0.000,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001, -0.000,  0.000, -0.000,  0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.001,  0.001,  0.000,  0.000,  0.000,  0.000,
    //            0.000 , 0.001 ,-0.000 ,-0.000 , 0.001 ,-0.000 , 0.000 ,-0.000 , 0.000 ,-0.000 ,-0.000 , 0.001 , 0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000,
    //            -0.000, -0.001,  0.000,  0.001, -0.001, -0.000, -0.001,  0.001,  0.000,  0.001,  0.001,  0.000,  1.708,  0.048,  0.784,  0.062,  0.042,  0.053,  0.077,  0.001, -0.061,  0.046, -0.019, -0.029,  0.000,  0.000,  0.000,  0.000,
    //            0.000 , 0.001 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 ,-0.000 , 0.048 , 5.001 ,-1.631 ,-0.036 , 0.144 , 0.040 , 0.036 , 0.016 ,-0.051 ,-0.067 ,-0.024 ,-0.005 , 0.000 , 0.000 , 0.000 , 0.000,
    //            -0.000, -0.001,  0.000,  0.000, -0.001, -0.000, -0.001,  0.000,  0.000,  0.000,  0.000, -0.000,  0.784, -1.631,  1.242,  0.057, -0.037,  0.018,  0.034, -0.017, -0.015,  0.058, -0.021, -0.029,  0.000,  0.000,  0.000,  0.000,
    //            0.000 , 0.000 , 0.000 , 0.001 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 , 0.062 ,-0.036 , 0.057 , 6.228 ,-0.014 , 0.932 , 0.059 , 0.053 ,-0.069 , 0.148 , 0.015 ,-0.031 , 0.000 , 0.000 , 0.000 , 0.000,
    //            -0.000,  0.000, -0.000, -0.000,  0.001,  0.000, -0.000,  0.000,  0.000, -0.000,  0.000,  0.000,  0.042,  0.144, -0.037, -0.014,  3.011,  0.986,  0.076,  0.030, -0.052, -0.027,  0.057,  0.051,  0.000,  0.000,  0.000,  0.000,
    //            -0.001, -0.001, -0.000,  0.001, -0.001,  0.000, -0.001,  0.001, -0.000,  0.001,  0.001,  0.000,  0.053,  0.040,  0.018,  0.932,  0.986,  0.885,  0.090,  0.044, -0.055,  0.057,  0.051, -0.003,  0.000,  0.000,  0.000,  0.000,
    //            -0.002, -0.003,  0.000,  0.002, -0.003, -0.000, -0.001,  0.002,  0.000,  0.002,  0.002, -0.000,  0.077,  0.036,  0.034,  0.059,  0.076,  0.090,  6.230,  0.139,  0.763,  0.013, -0.019, -0.024,  0.000,  0.000,  0.000,  0.000,
    //            0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.001 , 0.016 ,-0.017 , 0.053 , 0.030 , 0.044 , 0.139 , 3.130 ,-1.128 ,-0.010 , 0.131 , 0.018 , 0.000 , 0.000 , 0.000 , 0.000,
    //            -0.000, -0.001, -0.000,  0.000, -0.001, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,  0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055,  0.763, -1.128,  0.866, -0.022, -0.053,  0.007,  0.000,  0.000,  0.000,  0.000,
    //            -0.003, -0.004, -0.000,  0.003, -0.004, -0.000, -0.003,  0.003,  0.000,  0.003,  0.003,  0.000,  0.046, -0.067,  0.058,  0.148, -0.027,  0.057,  0.013, -0.010, -0.022,  2.437, -0.102,  0.938,  0.000,  0.000,  0.000,  0.000,
    //            -0.000, -0.000,  0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000,  0.001,  0.000, -0.019, -0.024, -0.021,  0.015,  0.057,  0.051, -0.019,  0.131, -0.053, -0.102,  4.944,  1.724,  0.000,  0.000,  0.000,  0.000,
    //            -0.001, -0.001,  0.000,  0.001, -0.001,  0.000, -0.001,  0.001, -0.000,  0.001,  0.001,  0.000, -0.029, -0.005, -0.029, -0.031,  0.051, -0.003, -0.024,  0.018,  0.007,  0.938,  1.724,  1.569,  0.000,  0.000,  0.000,  0.000,
    //            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000 , 0.000 , 0.000,
    //            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000 , 0.000,
    //            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000,
    //            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0;
    // /* A1 Worked */
    // _Cu <<   268.573,  -43.819, -147.211,
    //         -43.819 ,  92.949 ,  58.082,
    //         -147.211,   58.082,  302.120;
    // _QInit = _Qdig.asDiagonal();
    // _QInit +=  _B * _Cu * _B.transpose();// 参考书本(7.83)


    /* --------------------------lcc:start -------------------------- */
    _RInit <<
        0.0020,   -1.2209,   -0.8707,    0.0021,   -1.4539,    0.3631,    0.0021,   -1.2200,   -0.8711,    0.0014,   -1.4525,    0.3629,   -0.0118,    0.0103,    0.0013,   -0.0120,   -0.0234,    0.0042,   -0.0119,   -0.0225,   -0.0171,   -0.0110,   -0.0273,   -0.0130,    0.0000,    0.0000,    0.0000,    0.0000,
        -1.2209,  765.5080,  546.1835,   -1.2251,  911.8309, -227.5309,   -1.2278,  764.9412,  546.4145,   -1.0896,  911.0303, -227.4094,    7.3693,   -6.3426,   -0.8309,    7.3825,   14.6979,   -2.6369,    7.3562,   14.1501,   10.7008,    7.1891,   17.0501,    8.1558,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.8707,  546.1835,  393.1998,   -0.8745,  653.2483, -160.0706,   -0.8750,  545.7788,  393.3639,   -0.7886,  652.6778, -159.9855,    5.3102,   -4.5651,   -0.6496,    5.3160,   10.5041,   -1.8348,    5.2995,   10.0232,    7.7119,    5.1929,   12.1100,    5.9432,    0.0000,    0.0000,    0.0000,    0.0000,
        0.0021,   -1.2251,   -0.8745,    0.0024,   -1.4593,    0.3637,    0.0022,   -1.2240,   -0.8748,    0.0009,   -1.4577,    0.3635,   -0.0119,    0.0105,    0.0013,   -0.0123,   -0.0236,    0.0042,   -0.0121,   -0.0224,   -0.0171,   -0.0103,   -0.0277,   -0.0130,    0.0000,    0.0000,    0.0000,    0.0000,
        -1.4539,  911.8309,  653.2483,   -1.4593, 1088.1500, -269.2948,   -1.4616,  911.1556,  653.5230,   -1.3069, 1087.1972, -269.1512,    8.8176,   -7.5849,   -1.0329,    8.8304,   17.5204,   -3.1054,    8.8008,   16.7996,   12.8048,    8.6123,   20.2670,    9.8092,    0.0000,    0.0000,    0.0000,    0.0000,
        0.3631, -227.5309, -160.0706,    0.3637, -269.2948,   69.1010,    0.3654, -227.3627, -160.1398,    0.3170, -269.0565,   69.0638,   -2.1564,    1.8593,    0.2102,   -2.1625,   -4.3574,    0.8140,   -2.1533,   -4.2531,   -3.1307,   -2.0962,   -5.1033,   -2.3437,    0.0000,    0.0000,    0.0000,    0.0000,
        0.0021,   -1.2278,   -0.8750,    0.0022,   -1.4616,    0.3654,    0.0021,   -1.2267,   -0.8754,    0.0013,   -1.4601,    0.3653,   -0.0119,    0.0103,    0.0013,   -0.0121,   -0.0236,    0.0043,   -0.0120,   -0.0226,   -0.0171,   -0.0109,   -0.0275,   -0.0130,    0.0000,    0.0000,    0.0000,    0.0000,
        -1.2200,  764.9412,  545.7788,   -1.2240,  911.1556, -227.3627,   -1.2267,  764.3749,  546.0097,   -1.0891,  910.3557, -227.2413,    7.3638,   -6.3377,   -0.8303,    7.3769,   14.6870,   -2.6349,    7.3506,   14.1397,   10.6929,    7.1843,   17.0373,    8.1498,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.8711,  546.4145,  393.3639,   -0.8748,  653.5230, -160.1398,   -0.8754,  546.0097,  393.5281,   -0.7891,  652.9523, -160.0546,    5.3124,   -4.5669,   -0.6498,    5.3182,   10.5085,   -1.8356,    5.3017,   10.0275,    7.7151,    5.1952,   12.1151,    5.9457,    0.0000,    0.0000,    0.0000,    0.0000,
        0.0014,   -1.0896,   -0.7886,    0.0009,   -1.3069,    0.3170,    0.0013,   -1.0891,   -0.7891,    0.0031,   -1.3063,    0.3167,   -0.0104,    0.0086,    0.0014,   -0.0097,   -0.0209,    0.0036,   -0.0101,   -0.0203,   -0.0155,   -0.0126,   -0.0234,   -0.0121,    0.0000,    0.0000,    0.0000,    0.0000,
        -1.4525,  911.0303,  652.6778,   -1.4577, 1087.1972, -269.0565,   -1.4601,  910.3557,  652.9523,   -1.3063, 1086.2453, -268.9130,    8.8098,   -7.5781,   -1.0321,    8.8223,   17.5050,   -3.1026,    8.7930,   16.7849,   12.7936,    8.6057,   20.2489,    9.8008,    0.0000,    0.0000,    0.0000,    0.0000,
        0.3629, -227.4094, -159.9855,    0.3635, -269.1512,   69.0638,    0.3653, -227.2413, -160.0546,    0.3167, -268.9130,   69.0267,   -2.1553,    1.8584,    0.2101,   -2.1614,   -4.3551,    0.8135,   -2.1522,   -4.2508,   -3.1290,   -2.0949,   -5.1006,   -2.3424,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0118,    7.3693,    5.3102,   -0.0119,    8.8176,   -2.1564,   -0.0119,    7.3638,    5.3124,   -0.0104,    8.8098,   -2.1553,    0.0722,   -0.0618,   -0.0087,    0.0725,    0.1417,   -0.0248,    0.0723,    0.1350,    0.1040,    0.0702,    0.1634,    0.0802,    0.0000,    0.0000,    0.0000,    0.0000,
        0.0103,   -6.3426,   -4.5651,    0.0105,   -7.5849,    1.8593,    0.0103,   -6.3377,   -4.5669,    0.0086,   -7.5781,    1.8584,   -0.0618,    0.0535,    0.0068,   -0.0627,   -0.1220,    0.0220,   -0.0624,   -0.1160,   -0.0888,   -0.0595,   -0.1409,   -0.0683,    0.0000,    0.0000,    0.0000,    0.0000,
        0.0013,   -0.8309,   -0.6496,    0.0013,   -1.0329,    0.2102,    0.0013,   -0.8303,   -0.6498,    0.0014,   -1.0321,    0.2101,   -0.0087,    0.0068,    0.0040,   -0.0067,   -0.0161,   -0.0001,   -0.0068,   -0.0147,   -0.0154,   -0.0086,   -0.0174,   -0.0131,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0120,    7.3825,    5.3160,   -0.0123,    8.8304,   -2.1625,   -0.0121,    7.3769,    5.3182,   -0.0097,    8.8223,   -2.1614,    0.0725,   -0.0627,   -0.0067,    0.0748,    0.1422,   -0.0268,    0.0743,    0.1347,    0.1019,    0.0695,    0.1642,    0.0783,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0234,   14.6979,   10.5041,   -0.0236,   17.5204,   -4.3574,   -0.0236,   14.6870,   10.5085,   -0.0209,   17.5050,   -4.3551,    0.1417,   -0.1220,   -0.0161,    0.1422,    0.2824,   -0.0506,    0.1416,    0.2714,    0.2057,    0.1382,    0.3272,    0.1571,    0.0000,    0.0000,    0.0000,    0.0000,
        0.0042,   -2.6369,   -1.8348,    0.0042,   -3.1054,    0.8140,    0.0043,   -2.6349,   -1.8356,    0.0036,   -3.1026,    0.8135,   -0.0248,    0.0220,   -0.0001,   -0.0268,   -0.0506,    0.0120,   -0.0267,   -0.0492,   -0.0332,   -0.0242,   -0.0596,   -0.0242,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0119,    7.3562,    5.2995,   -0.0121,    8.8008,   -2.1533,   -0.0120,    7.3506,    5.3017,   -0.0101,    8.7930,   -2.1522,    0.0723,   -0.0624,   -0.0068,    0.0743,    0.1416,   -0.0267,    0.0739,    0.1343,    0.1016,    0.0699,    0.1634,    0.0781,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0225,   14.1501,   10.0232,   -0.0224,   16.7996,   -4.2531,   -0.0226,   14.1397,   10.0275,   -0.0203,   16.7849,   -4.2508,    0.1350,   -0.1160,   -0.0147,    0.1347,    0.2714,   -0.0492,    0.1343,    0.2634,    0.1968,    0.1320,    0.3162,    0.1487,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0171,   10.7008,    7.7119,   -0.0171,   12.8048,   -3.1307,   -0.0171,   10.6929,    7.7151,   -0.0155,   12.7936,   -3.1290,    0.1040,   -0.0888,   -0.0154,    0.1019,    0.2057,   -0.0332,    0.1016,    0.1968,    0.1544,    0.1016,    0.2370,    0.1194,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0110,    7.1891,    5.1929,   -0.0103,    8.6123,   -2.0962,   -0.0109,    7.1843,    5.1952,   -0.0126,    8.6057,   -2.0949,    0.0702,   -0.0595,   -0.0086,    0.0695,    0.1382,   -0.0242,    0.0699,    0.1320,    0.1016,    0.0724,    0.1581,    0.0787,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0273,   17.0501,   12.1100,   -0.0277,   20.2670,   -5.1033,   -0.0275,   17.0373,   12.1151,   -0.0234,   20.2489,   -5.1006,    0.1634,   -0.1409,   -0.0174,    0.1642,    0.3272,   -0.0596,    0.1634,    0.3162,    0.2370,    0.1581,    0.3811,    0.1795,    0.0000,    0.0000,    0.0000,    0.0000,
        -0.0130,    8.1558,    5.9432,   -0.0130,    9.8092,   -2.3437,   -0.0130,    8.1498,    5.9457,   -0.0121,    9.8008,   -2.3424,    0.0802,   -0.0683,   -0.0131,    0.0783,    0.1571,   -0.0242,    0.0781,    0.1487,    0.1194,    0.0787,    0.1795,    0.0935,    0.0000,    0.0000,    0.0000,    0.0000,
        0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    1000000,    0.0000,    0.0000,    0.0000,
        0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    1000000,    0.0000,    0.0000,
        0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    1000000,    0.0000,
        0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    0.0000,    1000000;



    _RInit = _RInit * 0.000001;
    _Rdig.setOnes();
    _Rdig = _Rdig * 0.0001 * 0.000001;
    // _Rdig = _Rdig * 0.0001 ;
    _RasD = _Rdig.asDiagonal();
    _RInit = _RInit + _RasD;



    _Cu <<     0.1997, -0.0505, -0.2113,
    -0.0505,  0.0379, -0.0056,
    -0.2113, -0.0056,  1.0054;
    _Cu = _Cu * 0.000001;
    _Qdig.setOnes();
    _Qdig = _Qdig * 0.0001 * 0.000001;
    // _Qdig(3) = 1 * 0.000001;
    // _Qdig(4) = 1 * 0.000001;
    _Qdig(5) = 1 * 0.000001;
    // _Qdig = _Qdig * 0.0001 ;
    _QInit = _Qdig.asDiagonal();
    _QInit +=  _B * _Cu * _B.transpose();// 参考书本(7.83)

    _QInit = _QInit * 1;

    // // std::cout<<" _RInit:\n "<< _RInit << std::endl;
    // // std::cout<<" _QInit:\n "<< _QInit << std::endl;
    /* --------------------------lcc:end -------------------------- */

    _RCheck  = new AvgCov(28, _estName + " R");
    _uCheck  = new AvgCov(3,  _estName + " u");

    _vxFilter = new LPFilter(_dt, 3.0);
    _vyFilter = new LPFilter(_dt, 3.0);
    _vzFilter = new LPFilter(_dt, 3.0);

    /* ROS odometry publisher */
    #ifdef COMPILE_WITH_MOVE_BASE
        _pub = _nh.advertise<nav_msgs::Odometry>("odom", 1);
    #endif  // COMPILE_WITH_MOVE_BASE
}

void Estimator::run(){
    _feetH.setZero();
    _feetPosGlobalKine = _robModel->getFeet2BPositions(*_lowState, FrameType::GLOBAL);
    _feetVelGlobalKine = _robModel->getFeet2BVelocities(*_lowState, FrameType::GLOBAL);

    _Q = _QInit;
    _R = _RInit;

    for(int i(0); i < 4; ++i){
        if((*_contact)(i) == 0){
            _Q.block(6+3*i, 6+3*i, 3, 3) = _largeVariance * I3;
            _R.block(12+3*i, 12+3*i, 3, 3) = _largeVariance * I3;
            _R(24+i, 24+i) = _largeVariance;
        }
        else{
            _trust = windowFunc((*_phase)(i), 0.2);
            _Q.block(6+3*i, 6+3*i, 3, 3) = (1 + (1-_trust)*_largeVariance) * _QInit.block(6+3*i, 6+3*i, 3, 3);
            _R.block(12+3*i, 12+3*i, 3, 3) = (1 + (1-_trust)*_largeVariance) * _RInit.block(12+3*i, 12+3*i, 3, 3);
            _R(24+i, 24+i) = (1 + (1-_trust)*_largeVariance) * _RInit(24+i, 24+i);
        }
        _feetPos2Body.segment(3*i, 3) = _feetPosGlobalKine.col(i);
        _feetVel2Body.segment(3*i, 3) = _feetVelGlobalKine.col(i);
    }

    _rotMatB2G = _lowState->getRotMat();
    // _u = _rotMatB2G * _lowState->getAcc() + _g;
    _u = _rotMatB2G * _lowState->getAcc();

    //lcc 20240604
    // if ( _lowState->userFunctionMode.function_test == true ){
    //     accOffset = -_u;
    // }
    // _u = _u + accOffset;

    _xhat = _A * _xhat + _B * _u;
    _yhat = _C * _xhat;
    _y << _feetPos2Body, _feetVel2Body, _feetH;


    // _RCheck->measure(_y);//lcc 20250602
    // _uCheck->measure(_u);//lcc 20250602
    // std::cout<<" _y:\n "<< _y << std::endl;
    // std::cout<<" _u:\n "<< _u << std::endl;

    _Ppriori = _A * _P * _A.transpose() + _Q;
    _S =  _R + _C * _Ppriori * _C.transpose();
    _Slu = _S.lu();
    _Sy = _Slu.solve(_y - _yhat);
    _Sc = _Slu.solve(_C);
    _SR = _Slu.solve(_R);
    _STC = (_S.transpose()).lu().solve(_C);
    _IKC = I18 - _Ppriori*_C.transpose()*_Sc;

    _xhat += _Ppriori * _C.transpose() * _Sy;
    _P =  _IKC * _Ppriori * _IKC.transpose()
        + _Ppriori * _C.transpose() * _SR * _STC * _Ppriori.transpose();

    // _vxFilter->addValue(_xhat(3));
    // _vyFilter->addValue(_xhat(4));
    // _vzFilter->addValue(_xhat(5));


    //lcc 20240604
    // if ( _lowState->userFunctionMode.function_test == true ){
    //     postionOffset = -_xhat.segment(0, 3) - Vec3(0, 0, _robModel-> _feetPosNormalStand(2));// 只有站起来才可以用
    //     velocityOffset = -_xhat.segment(3, 3);
    //     printf(" adadadada\n");
    // }
    // std::cout<<" getPosition(): "<< getPosition().transpose() << std::endl;
    // std::cout<<" getVelocity():\n "<< getVelocity().transpose() << std::endl;
    // std::cout<<" _u:\n "<< _u << std::endl;
    // std::cout<<" ODE_P:\n "<< ODE_P << std::endl;

    //lcc 20240603
    _xhat.segment(0, 3) = ODE_P;
    _xhat.segment(3, 3) = ODE_V;

    #ifdef COMPILE_WITH_MOVE_BASE
        if(_count % ((int)( 1.0/(_dt*_pubFreq))) == 0){
            _currentTime = ros::Time::now();
            /* tf */
            _odomTF.header.stamp = _currentTime;
            _odomTF.header.frame_id = "odom";
            _odomTF.child_frame_id  = "base";

            _odomTF.transform.translation.x = _xhat(0);
            _odomTF.transform.translation.y = _xhat(1);
            _odomTF.transform.translation.z = _xhat(2);
            _odomTF.transform.rotation.w = _lowState->imu.quaternion[0];
            _odomTF.transform.rotation.x = _lowState->imu.quaternion[1];
            _odomTF.transform.rotation.y = _lowState->imu.quaternion[2];
            _odomTF.transform.rotation.z = _lowState->imu.quaternion[3];

            _odomBroadcaster.sendTransform(_odomTF);

            /* odometry */
            _odomMsg.header.stamp = _currentTime;
            _odomMsg.header.frame_id = "odom";

            _odomMsg.pose.pose.position.x = _xhat(0);
            _odomMsg.pose.pose.position.y = _xhat(1);
            _odomMsg.pose.pose.position.z = _xhat(2);

            _odomMsg.pose.pose.orientation.w = _lowState->imu.quaternion[0];
            _odomMsg.pose.pose.orientation.x = _lowState->imu.quaternion[1];
            _odomMsg.pose.pose.orientation.y = _lowState->imu.quaternion[2];
            _odomMsg.pose.pose.orientation.z = _lowState->imu.quaternion[3];
            _odomMsg.pose.covariance = _odom_pose_covariance;

            _odomMsg.child_frame_id = "base";
            _velBody = _rotMatB2G.transpose() * _xhat.segment(3, 3);
            _wBody   = _lowState->imu.getGyro();
            _odomMsg.twist.twist.linear.x = _velBody(0);
            _odomMsg.twist.twist.linear.y = _velBody(1);
            _odomMsg.twist.twist.linear.z = _velBody(2);
            _odomMsg.twist.twist.angular.x = _wBody(0);
            _odomMsg.twist.twist.angular.y = _wBody(1);
            _odomMsg.twist.twist.angular.z = _wBody(2);
            _odomMsg.twist.covariance = _odom_twist_covariance;

            _pub.publish(_odomMsg);
            _count = 1;
        }
        ++_count;
    #endif  // COMPILE_WITH_MOVE_BASE
}

Vec3 Estimator::getPosition(){

    // return _xhat.segment(0, 3) + postionOffset; //lcc 20240604

    Vec3 z3;
    z3.setZero();
    return z3; //lcc 20240621  摆脱状态估计的依赖->_posbody
}

Vec3 Estimator::getVelocity(){
    // return _xhat.segment(3, 3) + velocityOffset; //lcc 20240604

    Vec3 z3;
    z3.setZero();
    return z3; //lcc 20240621  摆脱状态估计的依赖->_velbody
}

Vec3 Estimator::getFootPos(int i){
    return getPosition() + _lowState->getRotMat() * _robModel->getFootPosition(*_lowState, i, FrameType::BODY);
}

Vec34 Estimator::getFeetPos(){
    Vec34 feetPos;
    for(int i(0); i < 4; ++i){
        feetPos.col(i) = getFootPos(i);
    }
    return feetPos;
}

Vec34 Estimator::getFeetVel(){
    Vec34 feetVel = _robModel->getFeet2BVelocities(*_lowState, FrameType::GLOBAL);
    for(int i(0); i < 4; ++i){
        feetVel.col(i) += getVelocity();
    }
    return feetVel;
}

Vec34 Estimator::getPosFeet2BGlobal(){
    Vec34 feet2BPos;
    for(int i(0); i < 4; ++i){
        feet2BPos.col(i) = getFootPos(i) - getPosition();
    }
    return feet2BPos;
}





