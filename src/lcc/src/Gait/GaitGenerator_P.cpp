 
#include "Gait/GaitGenerator_P.h"

GaitGenerator_P::GaitGenerator_P(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase_hex), _contact(ctrlComp->contact_hex), 
                _robModel(ctrlComp->sixlegdogModel), _state(ctrlComp->lowState){
    _feetCal = new SupportFeetEndP(ctrlComp);
    _firstRun = true;
}

GaitGenerator_P::~GaitGenerator_P(){
}

void GaitGenerator_P::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){
    /* setGait： 得到 世界系下的速度 和 步高 */
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator_P::restart(){
    _firstRun = true;
    /* 将目标全局速度->setZero，因为落足点是根据速度来定的 */
    _vxyGoal.setZero();
    // std::cout<< "GaitGenerator_P restart!!! \n"<<std::endl;
}

void GaitGenerator_P::run(Vec36 &feetPos, Vec36 &feetVel){
    /* 先获得当前足端在world下的位置 */
    if(_firstRun){
        _firstRun = false;
        for (int i = 0; i < 6; i++)
            _startP.col(i) = _robModel->getFootPosition(*_state, i, FrameType::BODY);
    }

    /* 分别计算每条腿的规矩： 支撑相->轨迹为世界系下的足端位置； 摆动相->轨迹 由 落足点 和 摆线 构成*/
    for(int i(0); i<6; ++i){
        if((*_contact)(i) == 1){  // stand phase
            _startP.col(i) = ( _robModel->getFootPosition(*_state, i, FrameType::BODY));
            feetPos.col(i) = _startP.col(i);
            feetVel.col(i).setZero();
        }
        else{  // swing phase
            _endP.col(i) = _feetCal->calc_swing_fe(i, _vxyGoal, _dYawGoal, (*_phase)(i));
            feetPos.col(i) = getFootPos(i);
            feetVel.col(i) = getFootVel(i);
        }
    }

    // std::cout<<" (*_contact): \n"<< (*_contact).transpose() <<std::endl;
    // std::cout<<" (*_phase): \n"<< (*_phase).transpose() <<std::endl;
    _pastP = feetPos;
    _phasePast = *_phase;
}

Vec3 GaitGenerator_P::getFootPos(int i){  //这里就是摆动轨迹的计算
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i), _endP.col(i)(2));
    
    return footPos;
}

Vec3 GaitGenerator_P::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i), _endP.col(i)(2));

    return footVel;
}

float GaitGenerator_P::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start;
}

float GaitGenerator_P::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing();
}

float GaitGenerator_P::cycloidZPosition(float start, float h, float phase, float end){
    float phasePI = 2 * M_PI * phase;
    // return h*(1 - cos(phasePI))/2 + start;
    return h*(1 - cos(phasePI))/2 + start + phase * phase * end ;  //lcc
}

float GaitGenerator_P::cycloidZVelocity(float h, float phase, float end){
    float phasePI = 2 * M_PI * phase;
    // return h*M_PI * sin(phasePI) / _waveG->getTswing();
    return ( h*M_PI * sin(phasePI) + 2*phase*end) / _waveG->getTswing(); //lcc
}