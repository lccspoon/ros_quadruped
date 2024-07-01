 #include "Gait/SupportFeetEndP.h"

SupportFeetEndP::SupportFeetEndP(CtrlComponents *ctrlComp)
           : _est(ctrlComp->estimator), _lowState(ctrlComp->lowState),_Apla( ctrlComp->Apla),
             _robModel(ctrlComp->sixlegdogModel), _contact( ctrlComp->contact_hex), _phase(ctrlComp->phase_hex)
{
    _Tstance  = ctrlComp->waveGen->getTstance();
    _Tswing   = ctrlComp->waveGen->getTswing();

    _kx = 0.005;
    _ky = 0.005;
    _kyaw = 0.005;

    feetPosBody = _robModel->getFeetPosIdeal();
    for(int i(0); i<NUM_LEG_W; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) );
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i));
    }
}

SupportFeetEndP::~SupportFeetEndP(){}

Vec3 SupportFeetEndP::calc_support_fe(int legID, Vec2 vxyGoalGlobal, float dYawGoal, Vec3 P_Increment, Vec1_6 *terian_FootHold)

{

    _nextStep(0) = -vxyGoalGlobal(0) * _Tstance;
    _nextStep(1) = -vxyGoalGlobal(1) * _Tstance;
    _nextStep(2) = 0;

    //lcc
    double hx, hy;
    hx = _nextStep(0);
    hy = _nextStep(1);

    _yaw = 0;
    _dYaw = _lowState->getDYaw();
    // _nextYaw = -( _dYaw*(1-(*_phase)(legID))*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw) );
    _nextYaw = -( dYawGoal*_Tstance);

    // if (legID == 1 || legID==3 || legID==5)
    // {
    //     _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(0);
    //     _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(1)+0.2;
    //     _footPos = _nextStep;// getPosition->world系下质心位置
    //     #if PCONTROL_REFLEX_LIFE_DOWM == true
    //     (*terian_FootHold)(legID) = 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy + 0.0;
    //     #else
    //     _footPos(2) += 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy+0.2;
    //     #endif
    // }
    // else
    {
        _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(0);
        _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(1);
        _footPos = _nextStep;// getPosition->world系下质心位置
        #if PCONTROL_REFLEX_LIFE_DOWM == true
        (*terian_FootHold)(legID) = 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy + 0.0;
        #else
        _footPos(2) += 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy;
        #endif
    }


    if (legID == 1){
        // std::cout<<" _footPos-------------------- "<< legID <<"  :\n"<<_footPos.transpose() <<std::endl;
        // std::cout<<" ry "<< legID <<"  :\n"<<_feetRadius(legID) * sin(0 + _feetInitAngle(legID) + 0) <<std::endl;
    }
    
    // return _footPos + P_Increment;
    return _footPos;
}

Vec3 SupportFeetEndP::calc_swing_fe(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase, Vec3 P_Increment, Vec1_6 *terian_FootHold){

    _nextStep(0) = vxyGoalGlobal(0)*(1-phase)*_Tswing + vxyGoalGlobal(0)*_Tstance/2;
    _nextStep(1) = vxyGoalGlobal(1)*(1-phase)*_Tswing + vxyGoalGlobal(1)*_Tstance/2;
    // _nextStep(0) = 0 + vxyGoalGlobal(0)*_Tstance/2;
    // _nextStep(1) = 0 + vxyGoalGlobal(1)*_Tstance/2;
    _nextStep(2) = 0;

    //lcc
    double hx, hy;
    hx = _nextStep(0);
    hy = _nextStep(1);

    _yaw = 0;
    _dYaw = _lowState->getDYaw();
    // _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);
    _nextYaw = dYawGoal*_Tstance/2;

    // if (legID == 1 || legID==3 || legID==5)
    // {
    //     _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(0);
    //     _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(1)+0.2;
    //     _footPos = _nextStep;// getPosition->world系下质心位置
    //     #if PCONTROL_REFLEX_LIFE_DOWM == true
    //     (*terian_FootHold)(legID) = 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy + 0.0;
    //     #else
    //     _footPos(2) += 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy+0.2;
    //     #endif
    // }
    // else
    {
        _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(0);
        _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw) + P_Increment(1);
        _footPos = _nextStep;// getPosition->world系下质心位置
        #if PCONTROL_REFLEX_LIFE_DOWM == true
        (*terian_FootHold)(legID) = 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy + 0.0;
        #else
        _footPos(2) += 0 + (*_Apla)(1)*hx + (*_Apla)(2)*hy;
        #endif
    }

    // return _footPos + P_Increment;
    return _footPos;
}

