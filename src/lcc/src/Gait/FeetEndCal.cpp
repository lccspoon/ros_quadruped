/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(CtrlComponents *ctrlComp)
           : _est(ctrlComp->estimator), _lowState(ctrlComp->lowState),_Apla( ctrlComp->Apla),
             _robModel(ctrlComp->sixlegdogModel){
    _Tstance  = ctrlComp->waveGen->getTstance();
    _Tswing   = ctrlComp->waveGen->getTswing();

    _kx = 0.005;
    _ky = 0.005;
    _kyaw = 0.005;
    // _kyaw = 0.5;

    // double h= 0.31;
    // Vec34 _feetPos ;
    //  _feetPos <<
    // -0.00, -0.00, -0.00, -0.00,
    // -0.00, 0.00, -0.00, 0.00,
    // -h, -h, -h, -h;
    // Vec34 feetPosBody = _feetPos;

    Vec36 feetPosBody = _robModel->getFeetPosIdeal();
    for(int i(0); i<NUM_LEG_W; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) );
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

FeetEndCal::~FeetEndCal(){
}

Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase){
    _bodyVelGlobal = _est->getVelocity();
    _bodyWGlobal = _lowState->getGyroGlobal();

    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    _nextStep(2) = 0;

    //lcc
    double hx, hy;
    hx = _nextStep(0);
    hy = _nextStep(1);

    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);
    // _nextYaw =  _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    if (legID == 1)
    {

    // std::cout<<" rx "<< legID <<"  :\n"<<_feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw) <<std::endl;
    // std::cout<<" ry "<< legID <<"  :\n"<<_feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw) <<std::endl;

    // std::cout<<" cos "<< legID <<"  :\n"<< cos(_yaw + _feetInitAngle(legID) + _nextYaw) <<std::endl;
    // std::cout<<" sin "<< legID <<"  :\n"<<sin(_yaw + _feetInitAngle(legID) + _nextYaw) <<std::endl;

        // std::cout<<" _yaw "<< legID <<"  :\n"<< _yaw <<std::endl;
        // std::cout<<" _nextYaw "<< legID <<"  :\n"<<_nextYaw <<std::endl;
    }
    
    _footPos = _est->getPosition() + _nextStep;// getPosition->world系下质心位置
    _footPos(2) = 0.0;

    _footPos(2) = 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy;

    return _footPos;
}
