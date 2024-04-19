/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _state(ctrlComp->lowState){
    _feetCal = new FeetEndCal(ctrlComp);
    _firstRun = true;
}

GaitGenerator::~GaitGenerator(){
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){
    _firstRun = true;
    _vxyGoal.setZero();
    // std::cout<< "GaitGenerator restart!!! \n"<<std::endl;
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){
        _startP = _est->getFeetPos();
        _firstRun = false;
    }

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){
            if((*_phase)(i) < 0.5){
            // if((*_phase)(i) < 1){
                _startP.col(i) = _est->getFootPos(i);//返回足端在world下的坐标
            }
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else{
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i));

            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }

    _pastP = feetPos;
    _phasePast = *_phase;
    // std::cout<<" (*_contact3): "<< (*_contact).transpose() <<std::endl;
    // std::cout<<" (*_phase3): "<< (*_phase).transpose() <<std::endl;
    // std::cout<<" _endP: "<< _endP.row(2) <<std::endl;
    // std::cout<<" _gfoP: "<< _est->getFeetPos().row(2) <<std::endl;
    // std::cout<<" feetPos: "<< feetPos <<std::endl;

    // std::cout<<" _startP: "<< _startP <<std::endl;
}

Vec3 GaitGenerator::getFootPos(int i){  //这里就是摆动轨迹的计算
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i), _endP.col(i)(2));
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i), _endP.col(i)(2));

    return footVel;
}

float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator::cycloidZPosition(float start, float h, float phase, float end){
    float phasePI = 2 * M_PI * phase;
    // return h*(1 - cos(phasePI))/2 + start;
    return h*(1 - cos(phasePI))/2 + start + phase * phase * end ;  //lcc
}

float GaitGenerator::cycloidZVelocity(float h, float phase, float end){
    float phasePI = 2 * M_PI * phase;
    // return h*M_PI * sin(phasePI) / _waveG->getTswing();
    return ( h*M_PI * sin(phasePI) + 2*phase*end) / _waveG->getTswing(); //lcc
}