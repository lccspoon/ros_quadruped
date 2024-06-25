/*
    @author lcc
    @date 20240419
    @brief 算法： MPC+QP， 斜坡地形估计，等。 运动： 1m/s奔跑， 23度斜坡稳定上下，等。
    @cite 参考代码：unitree_guide,yy硕。 参考论文：yxy,mit。
*/
#include "FSM/State_MPC.h"
#include <iomanip>
#include <cmath>
State_A1MPC::State_A1MPC(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::A1MPC, "a1mpc"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase_hex), _Apla( ctrlComp->Apla),
              _contact(ctrlComp->contact_hex), _robModel(ctrlComp->robotModel), _sixlegdogModel(ctrlComp->sixlegdogModel), 
              _balCtrl(ctrlComp->balCtrl),_convMpc(ctrlComp->convMpc){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

// #ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
// #endif

    _vxLim = _sixlegdogModel->getRobVelLimitX();
    _vyLim = _sixlegdogModel->getRobVelLimitY();
    _wyawLim = _sixlegdogModel->getRobVelLimitYaw();

    /* ------------MPC start-------------- */
    //mpc init
    q_weights.resize(MPC_STATE_DIM);
    r_weights.resize(NUM_DOF);

    q_weights << 80.0, 80.0, 2.0,  //R P Y
            1, 30, 270.0,     // X Y Z
            1.0, 1.0, 30.0,    // WX WY WZ
            20.0, 30.0, 20.0,  // VX VY vz
            0.0;

    r_weights << 1e-5, 1e-5, 1e-6,
            1e-5, 1e-5, 1e-6,
            1e-5, 1e-5, 1e-6,
            1e-5, 1e-5, 1e-6,
            1e-5, 1e-5, 1e-6,
            1e-5, 1e-5, 1e-6;
    mpc_states.resize(MPC_STATE_DIM);
    mpc_init_counter = 0;
    /* ------------MPC end -------------- */
    _yaw = 0.0;
    _dYaw = 0.0;
    root_euler_d.setZero();

    _contact_te = new VecInt4;
}

State_A1MPC::~State_A1MPC(){
    delete _gait;
}

void State_A1MPC::enter(){
    // printf(" \n enter -> qp \n ");
    /* 一开始，设置期望的位置为实际位置；速度设置为0； */
    _pcd = _est->getPosition(); //一开始，将实际位置设置为目标位置。_pcd-> world系下，机身目标位置。
    // _pcd(2) = -_sixlegdogModel->getFeetPosIdeal()(2, 0);
    body_h = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0944;//lcc 20240604
    _pcd(2) = -_sixlegdogModel->getFeetPosIdeal()(2, 0) + 0.0944;//lcc 20240604

    /*-----命令清零-----*/
    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();
    _ctrlComp->ioInter->zeroCmdPanel();
    _gait->restart();
    printf(" State_A1MPC!\n ");
    _lowState->userValue.setZero();
}

void State_A1MPC::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
}

FSMStateName State_A1MPC::checkChange(){
    if(_lowState->userCmd == UserCommand::PASSIVE_1){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::FIXEDSTAND_2){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::A1MPC;
    }
}

void State_A1MPC::run(){
    // Rob State
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();

    #if TERRIANESTI_FOURLEG
        (*_contact_te)(0) = (*_contact)(0); 
        (*_contact_te)(1) = (*_contact)(1); 
        (*_contact_te)(2) = (*_contact)(4); 
        (*_contact_te)(3) = (*_contact)(5);
        _posFeet2BGlobal_te.block< 3, 1>( 0, 0) =  _posFeet2BGlobal.block< 3, 1>( 0, 0); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 1) =  _posFeet2BGlobal.block< 3, 1>( 0, 1); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 2) =  _posFeet2BGlobal.block< 3, 1>( 0, 4); 
        _posFeet2BGlobal_te.block< 3, 1>( 0, 3) =  _posFeet2BGlobal.block< 3, 1>( 0, 5); 
        // (*_contact_te)(0) = (*_contact)(0); 
        // (*_contact_te)(1) = (*_contact)(1); 
        // (*_contact_te)(2) = (*_contact)(2); 
        // (*_contact_te)(3) = (*_contact)(3);
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 0) =  _posFeet2BGlobal.block< 3, 1>( 0, 0); 
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 1) =  _posFeet2BGlobal.block< 3, 1>( 0, 1); 
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 2) =  _posFeet2BGlobal.block< 3, 1>( 0, 2); 
        // _posFeet2BGlobal_te.block< 3, 1>( 0, 3) =  _posFeet2BGlobal.block< 3, 1>( 0, 3); 
        // terr.terrain_adaptation( _posBody, _yawCmd, root_euler_d, _contact_te, _posFeet2BGlobal_te, _Apla);//lcc
    #else
        terr.terrain_adaptation( _posBody, _yawCmd, root_euler_d, _contact, _posFeet2BGlobal, _Apla);//lcc
    #endif

    /* 将键盘输入的_userValue转换为 需要的控制量：body系下的 目标速度、角速度 */
    getUserCmd();
    /* 继续，得到world系下的：机身目标速度、速度、期望姿态角yaw、dyaw */
    calcCmd();

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _q = vec36ToVec18(_lowState->getQ_Hex());
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcQPf();// QP力

    /*MPC*/
    if ( mpc_init_counter<PLAN_HORIZON *2){
        mpc_init_counter++;
    }
    calcGrf();// 计算MPC支反力
    if ( mpc_init_counter > PLAN_HORIZON + 1)  //
    {
        _forceFeetBody.block< 1, 6>( 0, 0) = _forceFeetBody.block< 1, 6>( 0, 0) * 2.5;
        _forceFeetBody.block< 2, 6>( 0, 0) = _forceFeetBody.block< 2, 6>( 0, 0) * 1;
        foot_forces_grf.block< 1, 6>( 0, 0) = foot_forces_grf.block< 1, 6>( 0, 0) * 2.5;
        foot_forces_grf.block< 2, 6>( 0, 0) = foot_forces_grf.block< 2, 6>( 0, 0) * 1;
        _tau = _sixlegdogModel->getTau(_q, _forceFeetBody * 0.6 + foot_forces_grf * 0.6 * 0.8); // qp + mpc
    }
    else
    {
        /*QP*/
        _tau = _sixlegdogModel->getTau(_q, _forceFeetBody * 1 + foot_forces_grf * 0); // qp + mpc
        // _tau = _sixlegdogModel->getTau(_q, _forceFeetBody); // qp + mpc
    }
    
    calcQQd();// 计算关节位置、速度
    _torqueCtrl();

    if(checkStepOrNot()){
        _ctrlComp->setStartWave();
    }else{
        _ctrlComp->setAllStance();
    }

    Vec18 tau_send;
    tau_send = _tau + torque18;
    _lowCmd->setTau(tau_send); //lcc 20240604
    // _lowCmd->setTau(torque18); //lcc 20240604
    // _lowCmd->setQ(vec34ToVec12(_qGoal));
    // _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<6; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }
}

bool State_A1MPC::checkStepOrNot(){
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

void State_A1MPC::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_lowState->userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -invNormalize(_lowState->userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    /* Turning */
    _dYawCmd = -invNormalize(_lowState->userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
    _dYawCmdPast = _dYawCmd;
}

void State_A1MPC::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody; //将机身速度映射到world系

    if ( _posBody(0) != 0 ||  _posBody(1) != 0 ||  _posBody(2) != 0){
        //origin
        _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
        _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));
        _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
        _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));
        _vCmdGlobal(2) = 0;
    }
    else{
        //lcc 20240621： 摆脱状态估计的依赖->_posbody
        _pcd(0) = _vCmdGlobal(0) * _ctrlComp->dt;
        _pcd(1) = _vCmdGlobal(1) * _ctrlComp->dt;
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
    _Rd = rpyToRotMat(root_euler_d(0), root_euler_d(1), _yawCmd)* eye3;//lcc
    _wCmdGlobal(2) = _dYawCmd;
}

void State_A1MPC::calcQPf(){
    _posError = _pcd - _posBody;
    /*--------------lcc start 20240604----------------*/
    Vec6 leg_deep;
    int leg_deep_num;
    leg_deep_num = 0;
    leg_deep.setZero();
    for (int i = 0; i < 6; i++)
    {   
        if((*_contact)(i) == 1) //stand
        {
            leg_deep(i) = _ctrlComp->sixlegdogModel->getFootPosition(*_lowState, i, FrameType::BODY)(2);
            leg_deep_num ++;
        }
    }
    if( (*_phase)(0) > 0.5 && (*_phase)(0) <= 0.6 )
    {
        body_h = -(leg_deep(0) + leg_deep(1) + leg_deep(2) + leg_deep(3) + leg_deep(4) + leg_deep(5) )/leg_deep_num + 0.0944;
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

    /* 对于 (*_contact)(i) == 0-> 处于swing的腿。使用PD来获得摆动力  */
    for(int i(0); i<6; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;//将足端力从world系转换到body系
    // _q = vec36ToVec18(_lowState->getQ_Hex());
    // _tau = _sixlegdogModel->getTau(_q, _forceFeetBody);
}

void State_A1MPC::calcQQd(){

    Vec36 _posFeet2B;
    _posFeet2B = _sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<6; ++i){
        // _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _pcd);//  可以用。 lcc 20240416
        // _posFeet2BGoal.col(i) = _G2B_RotMat * ( _est->getPosFeet2BGlobal().col(i) );// 可以用。 lcc 20240416
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    // _qGoal = vec12ToVec34(_sixlegdogModel->getQ(_posFeet2BGoal, FrameType::BODY));
    // _qdGoal = vec12ToVec34(_sixlegdogModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}


void State_A1MPC::_torqueCtrl(){

    _Kp = Vec3(3500, 3500, 3500).asDiagonal();
    _Kd = Vec3( 120,  120, 120).asDiagonal();
    Vec36 pos36;
    Vec36 vel36;
    Vec36 _targetPos36_o1;
    Vec36 _targetPos36_o2;
    Vec36 force36_o1;
    Vec36 force36_o2;
    Vec18 torque18_o1;
    Vec18 torque18_o2;
    Vec18 _q;

    pos36.setZero();
    vel36.setZero();
    force36_o1.setZero();
    force36_o2.setZero();
    _targetPos36_o1.setZero();
    _targetPos36_o2.setZero();
    torque18_o1.setZero();
    torque18_o2.setZero();
    _q.setZero();
    torque18.setZero();

    pos36 = _ctrlComp->sixlegdogModel->getFeet2BPositions(*_lowState,FrameType::BODY );
    vel36 = _ctrlComp->sixlegdogModel->getFeet2BVelocities(*_lowState,FrameType::BODY );

    _targetPos36_o2 = _posFeet2BGoal;
    _targetPos36_o1 = _est->getPosFeet2BGlobal();
    // std::cout<<" _posFeet2BGoal: \n"<< _posFeet2BGoal <<std::endl;
    // std::cout<<" _feetPosNormalStand: \n"<< _ctrlComp->sixlegdogModel->_feetPosNormalStand <<std::endl;
    for (int i = 0; i < 6; ++i){
        force36_o1.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o1.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
        force36_o2.block< 3, 1>( 0, i) =  _Kp*(_targetPos36_o2.block< 3, 1>( 0, i) - pos36.block< 3, 1>( 0, i) ) + _Kd*(-vel36.block< 3, 1>( 0, i) );
    }
    _q = vec36ToVec18(_lowState->getQ_Hex()); 
    torque18_o1 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o1);
    torque18_o2 = _ctrlComp->sixlegdogModel->getTau( _q, force36_o2);

    torque18 = torque18_o1 * 0 + torque18_o2 * 0.1;
}

void State_A1MPC::calcGrf(){

    ConvexMpc mpc_solver = ConvexMpc( q_weights, r_weights);
    mpc_solver.reset();

    Vec3 root_euler, root_ang_vel;
    root_euler = rotMatToRPY( _B2G_RotMat );
    root_ang_vel = _lowState->getGyroGlobal();
    mpc_states <<   root_euler(0), root_euler(1), root_euler(2), 
                    // _posBody(0), _posBody(1), _posBody(2), 
                    _posBody(0), _posBody(1), body_h, //lcc 20240604
                    root_ang_vel(0), root_ang_vel(1), root_ang_vel(2), 
                    _velBody(0), _velBody(1), _velBody(2), 
                    -9.8;
    // double mpc_dt = 0.002;
    double mpc_dt = _ctrlComp->dt;
    std::cout<<" body_h :"<< body_h <<std::endl;
    std::cout<<" _pcd(2) :"<< _pcd(2) <<std::endl;

     _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt;

    Vec3 root_lin_vel_d_world;
    root_lin_vel_d_world = _B2G_RotMat * _vCmdBody;
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        mpc_states_d.segment(i * MPC_STATE_DIM, MPC_STATE_DIM) <<
                root_euler_d(0),
                root_euler_d(1),
                root_euler(2) + _dYawCmd * mpc_dt * (i + 1),
                _posBody(0) +  root_lin_vel_d_world(0) * mpc_dt * (i + 1),
                _posBody(1) +  root_lin_vel_d_world(1) * mpc_dt * (i + 1),
                _pcd(2),
                _wCmdGlobal(0),
                _wCmdGlobal(1),
                _wCmdGlobal(2),
                _vCmdGlobal(0),
                _vCmdGlobal(1),
                0,
                -9.8;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    mpc_solver.calculate_A_mat_c(root_euler);

    auto t2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < PLAN_HORIZON; i++) {
        mpc_solver.calculate_B_mat_c(   _sixlegdogModel->getRobMass(),
                                        _sixlegdogModel->getRobInertial(),
                                        _B2G_RotMat,
                                        _posFeet2BGlobal);
        mpc_solver.state_space_discretization(mpc_dt);
        mpc_solver.B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(i * MPC_STATE_DIM, 0) = mpc_solver.B_mat_d;
    }

    // calculate QP matrices
    auto t3 = std::chrono::high_resolution_clock::now();
    bool contacts[NUM_LEG];
    VecInt6 contact_;
    contact_ = *_contact;
    for (int i = 0; i < NUM_LEG; i++){
        if( contact_(i) == 1 )
            contacts[i] = 1;
        else
            contacts[i] = 0;
    }
    mpc_solver.calculate_qp_mats( mpc_states, mpc_states_d, contacts);

    // solve
    auto t4 = std::chrono::high_resolution_clock::now();
    if (!solver.isInitialized()) {
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
        solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
        solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
        solver.data()->setHessianMatrix(mpc_solver.hessian);
        solver.data()->setGradient(mpc_solver.gradient);
        solver.data()->setLowerBound(mpc_solver.lb);
        solver.data()->setUpperBound(mpc_solver.ub);
        solver.initSolver();
    } else {
        solver.updateHessianMatrix(mpc_solver.hessian);
        solver.updateGradient(mpc_solver.gradient);
        solver.updateLowerBound(mpc_solver.lb);
        solver.updateUpperBound(mpc_solver.ub);
    }
    auto t5 = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto t6 = std::chrono::high_resolution_clock::now();

    // std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
    // std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
    // std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
    // std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
    // std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;
    // double total_time_ms = ms_double_1.count() + ms_double_2.count() + ms_double_3.count() + ms_double_4.count() + ms_double_5.count();
    // std::cout << "mpc cal A_mat_c: " << ms_double_1.count() << "ms" << std::endl;
    // std::cout << "mpc cal B_mat_d_list: " << ms_double_2.count() << "ms" << std::endl;
    // std::cout << "mpc cal qp mats: " << ms_double_3.count() << "ms" << std::endl;
    // std::cout << "mpc init time: " << ms_double_4.count() << "ms" << std::endl;
    // std::cout << "mpc solve time: " << ms_double_5.count() << "ms" << std::endl << std::endl;
    // std::cout << " calcGrf total time: " << total_time_ms << "ms" << std::endl;

    Eigen::VectorXd solution = solver.getSolution();

    for (int i = 0; i < NUM_LEG; ++i) {
        // if (!isnan(solution.segment<3>(i * 3).norm()))
            foot_forces_grf.block<3, 1>(0, i) = - _G2B_RotMat * solution.segment<3>(i * 3);
    }
}
