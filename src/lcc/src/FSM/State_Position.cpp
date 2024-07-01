 
#include "FSM/State_Position.h"
#include <iomanip>

State_Position::State_Position(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::POSITION, "position"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), _Apla( ctrlComp->Apla),
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), _sixlegdogModel(ctrlComp->sixlegdogModel), 
              _balCtrl(ctrlComp->balCtrl), _phase_hex(ctrlComp->phase_hex), _contact_hex(ctrlComp->contact_hex)
              {
    _gait = new GaitGenerator(ctrlComp);
    _gait_P = new GaitGenerator_P(ctrlComp);

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
    // _Kpp = Vec3(100, 30, 100).asDiagonal();
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

    _vxLim = _sixlegdogModel->getRobVelLimitX();
    _vyLim = _sixlegdogModel->getRobVelLimitY();
    _wyawLim = _sixlegdogModel->getRobVelLimitYaw();

    _contact_te = new VecInt4;
    _spf = new SupportFeetEndP(ctrlComp);
    _spt = new SupportTrajectory(ctrlComp);

    _rowMax = 20 * M_PI / 180;
    _rowMin = -_rowMax;
    _pitchMax = 15 * M_PI / 180;
    _pitchMin = -_pitchMax;
    _yawMax = 20 * M_PI / 180;
    _yawMin = -_yawMax;
    _heightMax = 0.18;
    _heightMin = -_heightMax + 0.05;

    adj_RPY_P.setZero();
    adj_RPY_P_past.setZero();

    _posFeet2BGoal_P_Increment.setZero();
    terian_FootHold = new Vec1_6;
    (*terian_FootHold).setZero();
}

State_Position::~State_Position(){
    delete _gait;
    delete _gait_P;
}

void State_Position::enter(){
    // printf(" \n enter -> qp \n ");
    /* 一开始，设置期望的位置为实际位置；速度设置为0； */
    _pcd = _est->getPosition(); //一开始，将实际位置设置为目标位置。_pcd-> world系下，机身目标位置。
    // _pcd(2) = -_sixlegdogModel->getFeetPosIdeal()(2, 0);
    body_h = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0944;//lcc 20240604
    _pcd(2) = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0944;//lcc 20240604

    // std::cout<<" _pcd :\n"<< _pcd.transpose() <<std::endl;

    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    /* 将所有UserCommand和UserValue设置为0 */
    _ctrlComp->ioInter->zeroCmdPanel();
    /* 将目标全局速度->setZero，因为落足点是根据速度来定的 */
    _gait->restart();
    _gait_P->restart();

    for(int i=0; i<18; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
    }
    _initVecOX = _ctrlComp->sixlegdogModel->getX(*_lowState); // P_b0_(0)
    _initVecXP = _ctrlComp->sixlegdogModel->getVecXP(*_lowState); // P_si

    _initFeetPos = _sixlegdogModel->getFeet2BPositions(*_lowState, FrameType::HIP);//QP

    _lowState->userValue.setZero();
}

void State_Position::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_Position::checkChange(){
    if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::POSITION;
    }
}

void State_Position::run(){
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

    #if TERRIANESTI_FOURLEG
        (*_contact_te)(0) = (*_contact_hex)(0); 
        (*_contact_te)(1) = (*_contact_hex)(1); 
        (*_contact_te)(2) = (*_contact_hex)(4); 
        (*_contact_te)(3) = (*_contact_hex)(5);
        _posFeet2BGlobal_te.block< 3, 1>( 0, 0) =  _posFeet2BGlobal.block< 3, 1>( 0, 0); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 1) =  _posFeet2BGlobal.block< 3, 1>( 0, 1); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 2) =  _posFeet2BGlobal.block< 3, 1>( 0, 4); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 3) =  _posFeet2BGlobal.block< 3, 1>( 0, 5); 
        // (*_contact_te)(0) = (*_contact_hex)(0); 
        // (*_contact_te)(1) = (*_contact_hex)(1); 
        // (*_contact_te)(2) = (*_contact_hex)(2); 
        // (*_contact_te)(3) = (*_contact_hex)(3);
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 0) =  _posFeet2BGlobal.block< 3, 1>( 0, 0); 
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 1) =  _posFeet2BGlobal.block< 3, 1>( 0, 1); 
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 2) =  _posFeet2BGlobal.block< 3, 1>( 0, 2); 
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 3) =  _posFeet2BGlobal.block< 3, 1>( 0, 3); 
        terr.terrain_adaptation( _posBody, _yawCmd, root_euler_d, _contact_te, _posFeet2BGlobal_te, _Apla);//lcc
    #else
        terr.terrain_adaptation( _posBody, _yawCmd, root_euler_d, _contact_hex, _posFeet2BGlobal, _Apla);//lcc
    #endif

    /* 将键盘输入的_userValue转换为 需要的控制量：body系下的 目标速度、角速度 */
    getUserCmd();
    /* 继续，得到world系下的：机身目标速度、速度、期望姿态角yaw、dyaw */
    calcCmd();

    /* setGait -> 设置世界系下的目标: vxyGoalGlobal, dYawGoal, gaitHeight*/
    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    /* run传的是指针，地址绑定，直接得到：世界系下足端目标位置和速度 */
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau(); // 计算关节力矩
    calcP();
    _torqueCtrl();//QP lcc 20240604

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    Vec18 tau_send;
    tau_send = _tau * 1 + torque18 * 1;
    _lowCmd->setTau( tau_send ); //lcc 20240602

    for(int i(0); i<6; ++i){
        if((*_contact_hex)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }
}

bool State_Position::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.01) ||
        (fabs(_vCmdBody(1)) > 0.01) ||
        (fabs(_posError(0)) > 0.04) ||
        (fabs(_posError(1)) > 0.04) ||
        (fabs(_velError(0)) > 0.025) ||
        (fabs(_velError(1)) > 0.025) ||
        (fabs(_dYawCmd) > 0.01) ){
        return true;
    }
    else{
        return false;
    }
}

void State_Position::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_lowState->userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_lowState->userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;
    
    /* Turning */
    _dYawCmd = -invNormalize(_lowState->userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_Position::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody; //将机身速度映射到world系

    if ( _posBody(0) != 0 ||  _posBody(1) != 0 ||  _posBody(2) != 0){
        //origin
        _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
        _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));
    }
    else{
        //lcc 20240621： 摆脱状态估计的依赖->_posbody
        _pcd(0) = _vCmdGlobal(0) * _ctrlComp->dt;
        _pcd(1) = _vCmdGlobal(1) * _ctrlComp->dt;
    }

    //lcc 20240624:在位置控制中，如果使用速度限制，会导致崩溃。
    if ( _velBody(0) != 0 ||  _velBody(1) != 0 ||  _velBody(2) != 0){
        //origin
        _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
        _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));
        _vCmdGlobal(2) = 0;
    }
    else
    {
        //lcc 20240621： 摆脱状态估计的依赖->_velbody
        _vCmdGlobal(0) = saturation(_vCmdGlobal(0), _vxLim);
        _vCmdGlobal(1) = saturation(_vCmdGlobal(1), _vyLim);
        _vCmdGlobal(2) = 0;
    }

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    // _Rd = rotz(_yawCmd);
    Eigen::Matrix3d eye3;
    eye3.setIdentity();
    // _Rd = rpyToRotMat(0, 0.2, _yawCmd)* eye3;//lcc
    // _yawCmd = saturation(_yawCmd, Vec2(_lowState->getYaw()-0.05, _lowState->getYaw()+0.05));
    // _dYawCmd = saturation(_dYawCmd, Vec2(_lowState->getDYaw()-0.2, _lowState->getDYaw()+0.2));
    _Rd = rpyToRotMat(root_euler_d(0), root_euler_d(1), _yawCmd)* eye3;//lcc
    // std::cout<<" root_euler_d :\n"<< root_euler_d.transpose() <<std::endl;
    _wCmdGlobal(2) = _dYawCmd;
}

void State_Position::calcTau(){
    _posError = _pcd - _posBody;
    /*--------------lcc start 20240604----------------*/
    Vec6 leg_deep;
    int leg_deep_num;
    leg_deep_num = 0;
    leg_deep.setZero();
    for (int i = 0; i < 6; i++)
    {   
        if((*_contact_hex)(i) == 1) //stand
        {
            leg_deep(i) = _ctrlComp->sixlegdogModel->getFootPosition(*_lowState, i, FrameType::BODY)(2);
            leg_deep_num ++;
        }
    }
    if( (*_phase_hex)(0) > 0.3 && (*_phase_hex)(0) <= 0.8 )
    {
        body_h = -(leg_deep(0) + leg_deep(1) + leg_deep(2) + leg_deep(3) + leg_deep(4) + leg_deep(5) )/leg_deep_num + 0.0944;
    }
    _posError(2) = _pcd(2) - body_h; // 使用接地腿的高度平均值作为机身实际高度，摆脱世界系下机身高度状态估计不准的难题
    // std::cout<<" body_h :"<< body_h <<std::endl;
    // std::cout<<" _pcd :\n"<< _pcd.transpose() <<std::endl;
    // std::cout<<" _posBody :\n"<< _posBody.transpose() <<std::endl;
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
    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact_hex);

    /* 对于 (*_contact)(i) == 0-> 处于swing的腿。使用PD来获得摆动力  */
    for(int i(0); i<6; ++i){
        if((*_contact_hex)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;//将足端力从world系转换到body系

    //lcc 20240617
    _forceFeetBody.block< 1, 6>( 0, 0) = _forceFeetBody.block< 1, 6>( 0, 0) * 5;
    _forceFeetBody.block< 2, 6>( 0, 0) = _forceFeetBody.block< 2, 6>( 0, 0) * 1;
    _q = vec36ToVec18(_lowState->getQ_Hex());
    _tau = _sixlegdogModel->getTau(_q, _forceFeetBody);
}

void State_Position::calcP(){
    //lcc 20240624: 位置控制的摆动轨迹
    _gait_P->setGait(_vCmdBody.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    // _gait_P->run(_posSwingLeg_P, _velSwingLeg_P);
    _gait_P->run(_posSwingLeg_P, _velSwingLeg_P, _posFeet2BGoal_P_Increment, terian_FootHold);
    //lcc 20240624: 位置控制的支撑轨迹
    _spt->setGait(_vCmdBody.segment(0,2), _wCmdGlobal(2), 0);
    // _spt->run(_posSupportLeg_P, _velSupportLeg_P);
    _spt->run(_posSupportLeg_P, _velSupportLeg_P, _posFeet2BGoal_P_Increment, terian_FootHold);
    for(int i(0); i<6; ++i){  
        if((*_contact_hex)(i) == 1){  //stand
            // _posFeet2BGoal_P.col(i) = _G2B_RotMat * (_posSupportLeg_P.col(i) - _posBody);
            _posFeet2BGoal_P.col(i) = 1 * (_posSupportLeg_P.col(i) );
            _velFeet2BGoal.col(i) = _G2B_RotMat * (_velSupportLeg_P.col(i) - _velBody); 
        }
        else if((*_contact_hex)(i) == 0){ //swing
                // _posFeet2BGoal_P.col(i) = _G2B_RotMat * (_posSwingLeg_P.col(i) - _posBody);
                _posFeet2BGoal_P.col(i) = 1 * (_posSwingLeg_P.col(i) );
                _velFeet2BGoal.col(i) = _G2B_RotMat * (_velSwingLeg_P.col(i) - _velBody); 
        }
    }

    //基于足端位置的姿态调整
    _initVecOX = _ctrlComp->sixlegdogModel->getFootPosition(*_lowState, 0, FrameType::BODY); // P_b0_(0)
    Vec3 x = _initVecOX;
    Vec36 vecXP, qLegs;
    qLegs = _lowState->getQ_Hex();
    for(int i(0); i < 6; ++i){
        vecXP.col(i) = _ctrlComp->sixlegdogModel->getFootPosition(*_lowState, i, FrameType::BODY) - x;
    }
    _initVecXP = vecXP;
    float row, pitch, yaw, height;
    Vec3 rpy;
    rpy = rotMatToRPY(_ctrlComp->lowState->getRotMat());
    adj_RPY_P << root_euler_d(0) - rpy(0), root_euler_d(1) - rpy(1), 0;
    // adj_RPY_P << root_euler_d(0) , root_euler_d(1) , 0;
    // std::cout<<" adj_RPY_P: \n"<< adj_RPY_P.transpose() <<std::endl;
    adj_RPY_P = 0.0 * adj_RPY_P_past + (1 - 0.0) * adj_RPY_P;
    // std::cout<<" adj_RPY_P affter: \n"<< adj_RPY_P.transpose() <<std::endl;
    adj_RPY_P_past = adj_RPY_P;
    row = invNormalize(adj_RPY_P(0), _rowMin, _rowMax);
    pitch = invNormalize(adj_RPY_P(1), _pitchMin, _pitchMax);
    yaw = -invNormalize(adj_RPY_P(2), _yawMin, _yawMax);
    // height = invNormalize(_lowState->userValue.ry, _heightMin, _heightMax) ;
    height = 0;
    for(int i(0); i < 6; ++i){
        if((*_contact_hex)(i) == 1){  //stand
        _posFeet2BGoal_P.col(i) = _posFeet2BGoal_P.col(i) + (_calcOP(row, pitch, yaw, height).col(i) - _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY ).col(i));
        }
    }
    _ctrlComp->lowCmd->setQ( _ctrlComp->sixlegdogModel->getQ( _posFeet2BGoal_P, FrameType::BODY) );
}

Vec36 State_Position::_calcOP(float row, float pitch, float yaw, float height){
    Vec3 vecXO = -_initVecOX;
    vecXO(2) += height;
    RotMat rotM = rpyToRotMat(row, pitch, yaw);
    HomoMat Tsb = homoMatrix(vecXO, rotM);
    HomoMat Tbs = homoMatrixInverse(Tsb);
    Vec4 tempVec6;
    Vec36 vecOP;
    for(int i(0); i<6; ++i){
        tempVec6 = Tbs * homoVec(_initVecXP.col(i));
        vecOP.col(i) = noHomoVec(tempVec6);
    }
    return vecOP;
}

void State_Position::_torqueCtrl(){

    _Kp = Vec3(3500, 3500, 3500).asDiagonal();
    _Kd = Vec3( 120,  120, 120).asDiagonal();
    Vec36 pos36;
    Vec36 vel36;
    Vec36 _targetPos36_o1;
    Vec36 _targetPos36_o2;
    Vec36 _targetPos36_o3;
    Vec36 force36_o1;
    Vec36 force36_o2;
    Vec36 force36_o3;
    Vec18 torque18_o1;
    Vec18 torque18_o2;
    Vec18 torque18_o3;
    Vec18 _q;

    pos36.setZero();
    vel36.setZero();
    force36_o1.setZero();
    force36_o2.setZero();
    force36_o3.setZero();
    _targetPos36_o1.setZero();
    _targetPos36_o2.setZero();
    _targetPos36_o3.setZero();
    torque18_o1.setZero();
    torque18_o2.setZero();
    torque18_o3.setZero();
    _q.setZero();
    torque18.setZero();

    pos36 = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY );
    vel36 = _ctrlComp->sixlegdogModel->getFeet2BVelocities(*_lowState,FrameType::BODY );

    for(int i(0); i<6; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
    }
    // _posFeet2BGoal.block< 1, 6>( 2, 0) = -_posFeet2BGoal.block< 1, 6>( 2, 0);
    // std::cout<<" _posBody: \n"<< _posBody <<std::endl;
    // std::cout<<" _posFeet2BGoal calcQQd: \n"<< _posFeet2BGoal <<std::endl;

    _targetPos36_o3 = _posFeet2BGoal_P;
    _targetPos36_o2 = _posFeet2BGoal;
    _targetPos36_o1 = _ctrlComp->sixlegdogModel->_feetPosNormalStand;
    // std::cout<<" _posFeet2BGoal: \n"<< _posFeet2BGoal <<std::endl;
    // std::cout<<" _feetPosNormalStand: \n"<< _ctrlComp->sixlegdogModel->_feetPosNormalStand <<std::endl;
    for (int i = 0; i < 6; ++i){
        force36_o1.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o1.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
        force36_o2.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o2.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
        force36_o3.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o3.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
    }
    _q = vec36ToVec18(_lowState->getQ_Hex()); 
    torque18_o1 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o1);
    torque18_o2 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o2);
    torque18_o3 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o3);
    // torque18 = torque18_o1 * 0.01 + torque18_o2 * 0.1 + torque18_o3 * 0; //力矩控制
    torque18 = torque18_o1 * 0.0005 + torque18_o2 * 0.005 + torque18_o3 * 1; //力位混合
    // torque18 = torque18_o1 * 0.0 + torque18_o2 * 0.0 + torque18_o3 * 1; //位置控制
}
