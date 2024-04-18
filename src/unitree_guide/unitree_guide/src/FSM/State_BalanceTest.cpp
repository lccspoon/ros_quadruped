/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_BalanceTest.h"

State_BalanceTest::State_BalanceTest(CtrlComponents *ctrlComp)
                  :FSMState(ctrlComp, FSMStateName::BALANCETEST, "balanceTest"),
                  _est(ctrlComp->estimator), _robModel(ctrlComp->robotModel), 
                  _balCtrl(ctrlComp->balCtrl), _contact(ctrlComp->contact){

    _xMax = 0.05;
    _xMin = -_xMax;
    _yMax = 0.05;
    _yMin = -_yMax;
    _zMax = 0.04;
    _zMin = -_zMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;

    _Kpp = Vec3(150, 150, 150).asDiagonal();
    _Kdp = Vec3(25, 25, 25).asDiagonal();

    _kpw = 200;
    _Kdw = Vec3(30, 30, 30).asDiagonal();
}

void State_BalanceTest::enter(){
    _pcdInit = _est->getPosition();// 返回world下的body位置
    _pcd = _pcdInit;
    _RdInit = _lowState->getRotMat();

    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();

    // printf(" lcc had removed : _lowCmd->setQ(_q)\n ");
}

void State_BalanceTest::run(){
    _userValue = _lowState->userValue;

    _pcd(0) = _pcdInit(0) + invNormalize(_userValue.ly, _xMin, _xMax);
    _pcd(1) = _pcdInit(1) - invNormalize(_userValue.lx, _yMin, _yMax);
    _pcd(2) = _pcdInit(2) + invNormalize(_userValue.ry, _zMin, _zMax);

    float yaw = invNormalize(_userValue.rx, _yawMin, _yawMax);
    _Rd = rpyToRotMat(0, 0, yaw)*_RdInit;

    _posBody = _est->getPosition();// 返回world下的body位置
    _velBody = _est->getVelocity();// 返回world下的body速度

    _B2G_RotMat = _lowState->getRotMat();// 返回body到world的旋转矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();

    calcTau();

    _lowCmd->setStableGain();
    _lowCmd->setTau(_tau);
    _lowCmd->setQ(_q);//lcc

    // Vec3 root_euler;
    // root_euler = rotMatToRPY( _B2G_RotMat );
    // std::cout<<" root_euler: \n"<< root_euler.transpose() * 180/3.1415926 <<std::endl;
}

void State_BalanceTest::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_BalanceTest::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::BALANCETEST;
    }
}

void State_BalanceTest::calcTau(){

    // @brief 基于力控的原地站立平衡控制的输入，V和W都为0
    // @param _pcd:期望body位置
    // @param _posBody:实际body位置
    // @param _Rd:期望姿态角
    _ddPcd = _Kpp*(_pcd - _posBody) + _Kdp * (Vec3(0, 0, 0) - _velBody);
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (Vec3(0, 0, 0) - _lowState->getGyroGlobal());

    _posFeet2BGlobal = _est->getPosFeet2BGlobal(); // world系下，foot_end到body的向量

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}