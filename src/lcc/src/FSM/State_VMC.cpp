/*
    @author lcc
    @date 20240523
*/
#include "FSM/State_VMC.h"
#include <iomanip>
#include <cmath>

State_VMC::State_VMC(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::VMC, "vmc"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), _Apla( ctrlComp->Apla),
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;
    root_euler_d.setZero();

// #ifdef ROBOT_TYPE_Go1
    // // _Kpp = Vec3(70, 70, 70).asDiagonal();
    // _Kdp = Vec3(10, 10, 10).asDiagonal();
    // _kpw = 780; 
    // _Kdw = Vec3(70, 70, 70).asDiagonal();
    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(10, 10, 10).asDiagonal();
// #endif

    //lcc tuning 20240604
    _Kpp = Vec3(30, 30, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 1000; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();

    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();


// #ifdef ROBOT_TYPE_A1
    // _Kpp = Vec3(20, 20, 100).asDiagonal();
    // _Kdp = Vec3(20, 20, 20).asDiagonal();
    // _kpw = 400;
    // _Kdw = Vec3(50, 50, 50).asDiagonal();
    // _KpSwing = Vec3(400, 400, 400).asDiagonal();
    // _KdSwing = Vec3(10, 10, 10).asDiagonal();
// #endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();
}

State_VMC::~State_VMC(){
    delete _gait;
}

void State_VMC::enter(){
    // printf(" \n enter -> vmc \n ");
    /* 一开始，设置期望的位置为实际位置；速度设置为0； */
    _pcd = _est->getPosition(); //一开始，将实际位置设置为目标位置。_pcd-> world系下，机身目标位置。
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);

    body_h = -_robModel->getFeetPosIdeal()(2, 0);//lcc 20240604

    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    /* 将所有UserCommand和UserValue设置为0 */
    _ctrlComp->ioInter->zeroCmdPanel();
    /* 将目标全局速度->setZero，因为落足点是根据速度来定的 */
    _gait->restart();

    _initFeetPos = _ctrlComp->robotModel->getFeet2BPositions(*_lowState, FrameType::HIP);//VMC

    // printf(" lcc had edited2!\n ");
    // std::cout<<" _vxLim: "<< _vxLim.transpose() <<std::endl;
    // std::cout<<" _vyLim: "<< _vyLim.transpose() <<std::endl;
    // std::cout<<" _wyawLim: "<< _wyawLim.transpose() <<std::endl;
}

void State_VMC::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_VMC::checkChange(){
    if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::VMC;
    }
}

void State_VMC::run(){
    // Rob State
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();//机身 到 世界 的变化矩阵
    _G2B_RotMat = _B2G_RotMat.transpose();//世界 到 机身 的变化矩阵

    // _lowState->userValue = _lowState->userValue;
    terr.terrain_adaptation( _posBody, _yawCmd, root_euler_d, _contact, _posFeet2BGlobal, _Apla);//lcc

    /* 将键盘输入的_userValue转换为 需要的控制量：body系下的 目标速度、角速度 */
    getUserCmd();
    /* 继续，得到world系下的：机身目标速度、速度、期望姿态角yaw、dyaw */
    calcCmd();

    /* setGait -> 设置世界系下的目标: vxyGoalGlobal, dYawGoal, gaitHeight*/
    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    /* 实际上，在FSM::run中一直在跑 _ctrlComp->runWaveGen(); */
    /* 而 phase->(0,1)和contact在构造时，已经将CtrlComponents *ctrlComp的 *contact 和 *phase 传给GaitGenerator */
    /* run传的是指针，地址绑定，直接得到：世界系下足端目标位置和速度 */
    /* 其实，在class GaitGenerator里面包含了class FeetEndCal，而 FeetEndCal 又 包含了 class Estimator。即需要根据世界系下，机身的实际速度、期望速度、相位进度、等来规划。*/
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau(); // 计算关节力矩
    calcQQd(); // 计算关节位置、速度
    _torqueCtrl();//VMC lcc 20240604

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    _lowCmd->setTau( _tau + torque12 * 0.1 ); //lcc 20240602
    // _lowCmd->setTau(_tau );
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }
}

bool State_VMC::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){
        return true;
    }
    else{
        return false;
    }
}

void State_VMC::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_lowState->userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_lowState->userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_lowState->userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_VMC::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody; //将机身速度映射到world系

    /* 把目标world系下的机身速度和位置，限制在真实的world系下的机身速度和位置附近 */
    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    // _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-1, _velBody(0)+1));
    // _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-1, _velBody(1)+1));
    // _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.5, _posBody(0) + 0.5));
    // _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.5, _posBody(1) + 0.5));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    // _Rd = rotz(_yawCmd);
    Eigen::Matrix3d eye3;
    eye3.setIdentity();
    // _Rd = rpyToRotMat(0, 0.2, _yawCmd)* eye3;//lcc
    _Rd = rpyToRotMat(root_euler_d(0), root_euler_d(1), _yawCmd)* eye3;//lcc
    _wCmdGlobal(2) = _dYawCmd;
}

void State_VMC::calcTau(){

    _posError = _pcd - _posBody;

    /*--------------lcc start 20240604----------------*/
    Vec4 leg_deep;
    int leg_deep_num;
    leg_deep_num = 0;
    leg_deep.setZero();
    for (int i = 0; i < 4; i++)
    {   
        if((*_contact)(i) == 1) //stand
        {
            leg_deep(i) = _ctrlComp->robotModel->getFootPosition(*_lowState, i, FrameType::BODY)(2);
            leg_deep_num ++;
        }
    }
    if( (*_phase)(0) > 0.5 && (*_phase)(0) <= 0.6 )
    {
        body_h = -(leg_deep(0) + leg_deep(1) + leg_deep(2) + leg_deep(3) )/leg_deep_num;
    }
    _posError(2) = _pcd(2) - body_h; // 使用接地腿的高度平均值作为机身实际高度，摆脱世界系下机身高度状态估计不准的难题
    // std::cout<<" body_h :"<< body_h <<std::endl;
    /*--------------lcc end 20240604----------------*/

    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    /* 对于 (*_contact)(i) == 1-> 处于stand的腿。使用QP来获得支撑力  */
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);
    // std::cout<<" _forceFeetGlobal1: \n"<< _forceFeetGlobal <<std::endl;
    // std::cout<<" (*_contact): \n"<< (*_contact) <<std::endl;
    // std::cout<<" _posError: \n"<< _posError <<std::endl;
    // std::cout<<" _pcd: \n"<< _pcd <<std::endl;
    // std::cout<<" _posBody: \n"<< _posBody <<std::endl;

    /* 对于 (*_contact)(i) == 0-> 处于swing的腿。使用PD来获得摆动力  */
    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }
    // std::cout<<" _forceFeetGlobal2: \n"<< _forceFeetGlobal <<std::endl;

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;//将足端力从world系转换到body系
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_VMC::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        /* 世界系下： pFeet2B = pFeet2Global - _pcd */
        // _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _pcd);//  可以用。 lcc 20240416
        // _posFeet2BGoal.col(i) = _G2B_RotMat * ( _est->getPosFeet2BGlobal().col(i) );// 可以用。 lcc 20240416
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);

        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }

    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}


void State_VMC::_torqueCtrl(){

    _Kp = Vec3(5000, 5000, 5000).asDiagonal();
    _Kd = Vec3( 200,  200, 200).asDiagonal();

    Vec34 pos34;
    Vec34 vel34;
    Vec34 _targetPos34;
    Vec34 force34;
    pos34.setZero();
    vel34.setZero();
    force34.setZero();
    _targetPos34.setZero();
    torque12.setZero();

    pos34 = _ctrlComp->robotModel->getFeet2BPositions(*_lowState,FrameType::BODY );
    vel34 = _ctrlComp->robotModel->getFeet2BVelocities(*_lowState,FrameType::BODY );

    _targetPos34 = _posFeet2BGoal;

    for (int i = 0; i < 4; ++i){
        force34.block< 3, 1>( 0, i) =  _Kp*(_targetPos34.block< 3, 1>( 0, i) - pos34.block< 3, 1>( 0, i) ) + _Kd*(-vel34.block< 3, 1>( 0, i) );
    }

    Vec12 _q = vec34ToVec12(_lowState->getQ()); 
    torque12 = _ctrlComp->robotModel->getTau( _q, force34);
}
