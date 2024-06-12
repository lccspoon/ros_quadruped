#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

Vec3 ODE_P;
Vec3 ODE_V;

void RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

//IOROS::IOROS():IOInterface(){} 表示在构造 IOROS 类的对象时，首先调用其基类 IOInterface 的默认构造函数。
//这是一种初始化基类的常见方式，尤其是当基类有一个显式的构造函数时。
IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;

    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(4); // 4 threads
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();   

    signal(SIGINT, RosShutDown);

    // 由于 KeyBoard 类继承自 CmdPanel 类，因此 KeyBoard 对象也被视为一种 CmdPanel 对象。
    // 这就是继承的基本概念，子类对象可以赋值给父类指针或引用。
    cmdPanel = new KeyBoard();
}

IOROS::~IOROS(){
    delete cmdPanel;
    ros::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
    state->userFunctionMode = cmdPanel->getUserFunctionMode();// lcc 20250601
    // state->userFunctionMode_p->function_test = cmdPanel->getUserFunctionMode().function_test;
    // state->userFunctionMode_p = &cmdPanel->getUserFunctionMode();
    // retSimOdeBodyP();
    // retSimOdeBodyV();
    ODE_P = retSimOdeBodyP();
    ODE_V = retSimOdeBodyV();
    // std::cout << "retSimOdeBodyP:\n" <<retSimOdeBodyP()<< std::endl;
    // std::cout << "retSimOdeBodyV:\n" <<retSimOdeBodyV()<< std::endl;

}

void IOROS::initSend(){
    // ROS publisher
    pub_joint_cmd[0] = nh.advertise<std_msgs::Float64>("/sixlegdog/rfj1_jointcc/command", 1000);
    pub_joint_cmd[1] = nh.advertise<std_msgs::Float64>("/sixlegdog/rfj2_jointcc/command", 1000);
    pub_joint_cmd[2] = nh.advertise<std_msgs::Float64>("/sixlegdog/rfj3_jointcc/command", 1000);

    pub_joint_cmd[3] = nh.advertise<std_msgs::Float64>("/sixlegdog/lfj5_jointcc/command", 1000);
    pub_joint_cmd[4] = nh.advertise<std_msgs::Float64>("/sixlegdog/lfj6_jointcc/command", 1000);
    pub_joint_cmd[5] = nh.advertise<std_msgs::Float64>("/sixlegdog/lfj7_jointcc/command", 1000);

    pub_joint_cmd[6] = nh.advertise<std_msgs::Float64>("/sixlegdog/rmj9_jointcc/command", 1000);
    pub_joint_cmd[7] = nh.advertise<std_msgs::Float64>("/sixlegdog/rmj10_jointcc/command", 1000);
    pub_joint_cmd[8] = nh.advertise<std_msgs::Float64>("/sixlegdog/rmj11_jointcc/command", 1000);

    pub_joint_cmd[9] = nh.advertise<std_msgs::Float64>("/sixlegdog/lmj13_jointcc/command", 1000);
    pub_joint_cmd[10] = nh.advertise<std_msgs::Float64>("/sixlegdog/lmj14_jointcc/command", 1000);
    pub_joint_cmd[11] = nh.advertise<std_msgs::Float64>("/sixlegdog/lmj15_jointcc/command", 1000);

    pub_joint_cmd[12] = nh.advertise<std_msgs::Float64>("/sixlegdog/rbj17_jointcc/command", 1000);
    pub_joint_cmd[13] = nh.advertise<std_msgs::Float64>("/sixlegdog/rbj18_jointcc/command", 1000);
    pub_joint_cmd[14] = nh.advertise<std_msgs::Float64>("/sixlegdog/rbj19_jointcc/command", 1000);

    pub_joint_cmd[15] = nh.advertise<std_msgs::Float64>("/sixlegdog/lbj21_jointcc/command", 1000);
    pub_joint_cmd[16] = nh.advertise<std_msgs::Float64>("/sixlegdog/lbj22_jointcc/command", 1000);
    pub_joint_cmd[17] = nh.advertise<std_msgs::Float64>("/sixlegdog/lbj23_jointcc/command", 1000);

    printf(" IOROS ros inti.\n");
}

// void IOROS::doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
Eigen::Matrix<double, LEG_DOF_W, NUM_LEG_W> sub_joint_p_local, sub_joint_p_local_temp;
Eigen::Matrix<double, LEG_DOF_W, NUM_LEG_W> sub_joint_v_local, sub_joint_v_local_temp;
Eigen::Matrix<double, LEG_DOF_W, NUM_LEG_W> sub_joint_t_local, sub_joint_t_local_temp;
int sub_joint_state_seq_local;
void doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
{
    sub_joint_state_seq_local = sub_joint_states->header.seq;
    for (int i = 0; i < NUM_DOF_W; i++)
    {
        sub_joint_p_local_temp(i) = sub_joint_states->position[i];
        sub_joint_v_local_temp(i) = sub_joint_states->velocity[i];
        sub_joint_t_local_temp(i) = sub_joint_states->effort[i];
    }

    //  0~12: RF LF RB LB 
    // sub_joint_p_local.block<3,1>(0,0) = sub_joint_p_local_temp.block<3,1>(0,3);
    // sub_joint_p_local.block<3,1>(0,1) = sub_joint_p_local_temp.block<3,1>(0,1);
    // sub_joint_p_local.block<3,1>(0,2) = sub_joint_p_local_temp.block<3,1>(0,2);
    // sub_joint_p_local.block<3,1>(0,3) = sub_joint_p_local_temp.block<3,1>(0,0);

    // $ rostopic echo /sixlegdog/joint_states
    // name: 
    //   - lbj21
    //   - lbj22
    //   - lbj23
    //   - lfj5
    //   - lfj6
    //   - lfj7
    //   - lmj13
    //   - lmj14
    //   - lmj15
    //   - rbj17
    //   - rbj18
    //   - rbj19
    //   - rfj1
    //   - rfj2
    //   - rfj3
    //   - rmj10
    //   - rmj11
    //   - rmj9


   //  0~18: RF LF RM LM RB LB 
    sub_joint_p_local.block<3,1>(0,0) = sub_joint_p_local_temp.block<3,1>(0,4);//rf
    sub_joint_p_local.block<3,1>(0,1) = sub_joint_p_local_temp.block<3,1>(0,1);//lf

    // sub_joint_p_local.block<3,1>(0,2) = sub_joint_p_local_temp.block<3,1>(0,5);//rm lcc20240610: warning -> 一定要严格对照接收数据的对应关节的顺序，不要搞错！！！
    sub_joint_p_local(6) = sub_joint_p_local_temp(17);
    sub_joint_p_local(7) = sub_joint_p_local_temp(15);
    sub_joint_p_local(8) = sub_joint_p_local_temp(16);

    sub_joint_p_local.block<3,1>(0,3) = sub_joint_p_local_temp.block<3,1>(0,2);//lm
    sub_joint_p_local.block<3,1>(0,4) = sub_joint_p_local_temp.block<3,1>(0,3);//rb
    sub_joint_p_local.block<3,1>(0,5) = sub_joint_p_local_temp.block<3,1>(0,0);//lb
    // std::cout << "sub_joint_p_local\n" << sub_joint_p_local<< std::endl;

    //  0~12: RF LF RB LB 
    // sub_joint_v_local.block<3,1>(0,0) = sub_joint_v_local_temp.block<3,1>(0,3);
    // sub_joint_v_local.block<3,1>(0,1) = sub_joint_v_local_temp.block<3,1>(0,1);
    // sub_joint_v_local.block<3,1>(0,2) = sub_joint_v_local_temp.block<3,1>(0,2);
    // sub_joint_v_local.block<3,1>(0,3) = sub_joint_v_local_temp.block<3,1>(0,0);

   //  0~18: RF LF RM LM RB LB 
    sub_joint_v_local.block<3,1>(0,0) = sub_joint_v_local_temp.block<3,1>(0,4);
    sub_joint_v_local.block<3,1>(0,1) = sub_joint_v_local_temp.block<3,1>(0,1);

    // sub_joint_v_local.block<3,1>(0,2) = sub_joint_v_local_temp.block<3,1>(0,5);
    sub_joint_v_local(6) = sub_joint_v_local_temp(17);
    sub_joint_v_local(7) = sub_joint_v_local_temp(15);
    sub_joint_v_local(8) = sub_joint_v_local_temp(16);

    sub_joint_v_local.block<3,1>(0,3) = sub_joint_v_local_temp.block<3,1>(0,2);
    sub_joint_v_local.block<3,1>(0,4) = sub_joint_v_local_temp.block<3,1>(0,3);
    sub_joint_v_local.block<3,1>(0,5) = sub_joint_v_local_temp.block<3,1>(0,0);
    // std::cout << "sub_joint_p_local:\n" <<sub_joint_p_local<< std::endl;
    // std::cout << "sub_joint_v_local:\n" <<sub_joint_v_local<< std::endl;

    //  0~12: RF LF RB LB 
    // sub_joint_t_local.block<3,1>(0,0) = sub_joint_t_local_temp.block<3,1>(0,3);
    // sub_joint_t_local.block<3,1>(0,1) = sub_joint_t_local_temp.block<3,1>(0,1);
    // sub_joint_t_local.block<3,1>(0,2) = sub_joint_t_local_temp.block<3,1>(0,2);
    // sub_joint_t_local.block<3,1>(0,3) = sub_joint_t_local_temp.block<3,1>(0,0);

   //  0~18: RF LF RM LM RB LB 
    sub_joint_t_local.block<3,1>(0,0) = sub_joint_t_local_temp.block<3,1>(0,4);
    sub_joint_t_local.block<3,1>(0,1) = sub_joint_t_local_temp.block<3,1>(0,1);

    // sub_joint_t_local.block<3,1>(0,2) = sub_joint_t_local_temp.block<3,1>(0,5);
    sub_joint_t_local(6) = sub_joint_t_local_temp(17);
    sub_joint_t_local(7) = sub_joint_t_local_temp(15);
    sub_joint_t_local(8) = sub_joint_t_local_temp(16);

    sub_joint_t_local.block<3,1>(0,3) = sub_joint_t_local_temp.block<3,1>(0,2);
    sub_joint_t_local.block<3,1>(0,4) = sub_joint_t_local_temp.block<3,1>(0,3);
    sub_joint_t_local.block<3,1>(0,5) = sub_joint_t_local_temp.block<3,1>(0,0);

    //旋转方向调整->p
    sub_joint_p_local(2) = -sub_joint_p_local(2);
    sub_joint_p_local(4) = -sub_joint_p_local(4);

    sub_joint_p_local(6) = -sub_joint_p_local(6);
    sub_joint_p_local(8) = -sub_joint_p_local(8);

    sub_joint_p_local(9) = -sub_joint_p_local(9);
    sub_joint_p_local(10) = -sub_joint_p_local(10);

    sub_joint_p_local(12) = -sub_joint_p_local(12);
    sub_joint_p_local(14) = -sub_joint_p_local(14);

    sub_joint_p_local(15) = -sub_joint_p_local(15);
    sub_joint_p_local(16) = -sub_joint_p_local(16);

    //旋转方向调整->v
    // sub_joint_v_local(1) = -sub_joint_v_local(1);
    sub_joint_v_local(2) = -sub_joint_v_local(2);
    sub_joint_v_local(4) = -sub_joint_v_local(4);

    sub_joint_v_local(6) = -sub_joint_v_local(6);
    sub_joint_v_local(8) = -sub_joint_v_local(8);

    sub_joint_v_local(9) = -sub_joint_v_local(9);
    sub_joint_v_local(10) = -sub_joint_v_local(10);

    sub_joint_v_local(12) = -sub_joint_v_local(12);
    sub_joint_v_local(14) = -sub_joint_v_local(14);

    sub_joint_v_local(15) = -sub_joint_v_local(15);
    sub_joint_v_local(16) = -sub_joint_v_local(16);

    //旋转方向调整->t
    // sub_joint_t_local(1) = -sub_joint_t_local(1);
    sub_joint_t_local(2) = -sub_joint_t_local(2);
    sub_joint_t_local(4) = -sub_joint_t_local(4);

    sub_joint_t_local(6) = -sub_joint_t_local(6);
    sub_joint_t_local(8) = -sub_joint_t_local(8);

    sub_joint_t_local(9) = -sub_joint_t_local(9);
    sub_joint_t_local(10) = -sub_joint_t_local(10);

    sub_joint_t_local(12) = -sub_joint_t_local(12);
    sub_joint_t_local(14) = -sub_joint_t_local(14);

    sub_joint_t_local(15) = -sub_joint_t_local(15);
    sub_joint_t_local(16) = -sub_joint_t_local(16);

}

Eigen::Matrix<double,4,1> sub_imu_orie_local;
Eigen::Matrix<double,3,1> sub_imu_ang_v_local;
Eigen::Matrix<double,3,1> sub_imu_lin_a_local;
int sub_imu_seq_local;
void doMsg_yobotics_imu_msg(const sensor_msgs::Imu::ConstPtr& sub_imu_msg){
    sub_imu_seq_local = sub_imu_msg->header.seq;

    sub_imu_orie_local << sub_imu_msg->orientation.x, sub_imu_msg->orientation.y, sub_imu_msg->orientation.z, sub_imu_msg->orientation.w;
    sub_imu_ang_v_local << sub_imu_msg->angular_velocity.x, sub_imu_msg->angular_velocity.y, sub_imu_msg->angular_velocity.z;
    sub_imu_lin_a_local << sub_imu_msg->linear_acceleration.x, sub_imu_msg->linear_acceleration.y, sub_imu_msg->linear_acceleration.z;

    // std::cout << sub_imu_ang_v_local << std::endl;
}

Eigen::Matrix<double,3,1> sub_odo_pos_local;
Eigen::Matrix<double,3,1> sub_odo_lin_twist_local;
void doMsg_nav_odometry(const nav_msgs::Odometry::ConstPtr& sub_nav_odometry){
    sub_odo_pos_local << sub_nav_odometry->pose.pose.position.x, sub_nav_odometry->pose.pose.position.y, 
                    sub_nav_odometry->pose.pose.position.z;
    sub_odo_lin_twist_local << sub_nav_odometry->twist.twist.linear.x, sub_nav_odometry->twist.twist.linear.y, 
                         sub_nav_odometry->twist.twist.linear.z;
    // std::cout << sub_odo_pos_local << std::endl;
}

void IOROS::initRecv(){
    sub_joint_states = nh.subscribe<sensor_msgs::JointState>("/sixlegdog/joint_states",1,doMsg_yobotics_joint_states);
    sub_imu_msg = nh.subscribe<sensor_msgs::Imu>("/imu",1,doMsg_yobotics_imu_msg);
    sub_nav_odometry = nh.subscribe<nav_msgs::Odometry>("/POS_COM",1,doMsg_nav_odometry);
}

Eigen::Matrix<double, 3, 1> IOROS::retSimOdeBodyP(){
    return sub_odo_pos_local;
}

Eigen::Matrix<double, 3, 1> IOROS::retSimOdeBodyV(){
    return sub_odo_lin_twist_local;
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    std_msgs::Float64 pub_data[NUM_DOF_W];

    Vec18 init_tau;
    init_tau.setZero();
    init_tau(2) = -0.1;
    init_tau(5) = -0.1;
    init_tau(8) = -0.1;
    init_tau(11) = -0.1;
    init_tau(14) = -0.1;
    init_tau(17) = -0.1;

    Vec18 init_pos;
    init_pos.setZero();
    init_pos(0) = -1.57;
    init_pos(2) = 1.57;

    init_pos(3) = 1.57;
    init_pos(5) = -1.57;

    //rm
    init_pos(6) = 1.57;
    init_pos(8) = 1.57;

    init_pos(9) = -1.57;
    init_pos(11) = -1.57;
    // init_pos(11) = -0.157;

    init_pos(12) = 1.57;
    init_pos(14) = 1.57;

    init_pos(15) = -1.57;
    init_pos(17) = -1.57;

    if (cmdPanel->userFunctionMode.function_test == true )
    {
        // if (IS_THIS_A_HEXAPOD)
            // printf("\nsdsfas\n");
        
        for(int m(0); m < NUM_DOF_W; ++m){
            // pub_data[m].data = init_pos(m);
            pub_data[m].data = init_tau(m)*1;
        }
    }
    else
    {
        init_pos.setZero();
        for(int m(0); m < NUM_DOF_W; ++m){
            // pub_data[m].data = init_pos(m);
            // pub_data[m].data = lowCmd->motorCmd[m].q;
            pub_data[m].data = lowCmd->motorCmd[m].tau;
        }
    }

    // pub_data[1].data = -pub_data[1].data; 
    pub_data[2].data = -pub_data[2].data; 

    pub_data[4].data = -pub_data[4].data; 

    pub_data[6].data = -pub_data[6].data; 
    // pub_data[7].data = -pub_data[7].data; 
    pub_data[8].data = -pub_data[8].data; 

    pub_data[9].data = -pub_data[9].data; 
    pub_data[10].data = -pub_data[10].data; 

    pub_data[12].data = -pub_data[12].data; 
    // pub_data[13].data = -pub_data[13].data; 
    pub_data[14].data = -pub_data[14].data; 

    pub_data[15].data = -pub_data[15].data; 
    pub_data[16].data = -pub_data[16].data; 

    for(int m(0); m < NUM_DOF_W; ++m){
        // pub_data[m].data = 0;
        pub_joint_cmd[m].publish(pub_data[m]);
    }
    // std::cout << "retSimJointP:\n" <<retSimJointP()<< std::endl;
    ros::spinOnce();
}

void IOROS::recvState(LowlevelState *state){

    for(int i(0); i < NUM_DOF_W; ++i){
        state->motorState[i].q = sub_joint_p_local(i);
        state->motorState[i].dq = sub_joint_v_local(i);
        state->motorState[i].tauEst = sub_joint_t_local(i);
    }
    for(int i(0); i < 3; ++i){
        state->imu.accelerometer[i] = sub_imu_lin_a_local(i);
        state->imu.gyroscope[i] = sub_imu_ang_v_local(i);
    }
    // Note: state->imu.quaternion:  w, x, y, z
    //   geometry_msgs/Quaternion orientation
    //   float64 x
    //   float64 y
    //   float64 z
    //   float64 w

    state->imu.quaternion[0] = sub_imu_orie_local[3];
    state->imu.quaternion[1] = sub_imu_orie_local[0];
    state->imu.quaternion[2] = sub_imu_orie_local[1];
    state->imu.quaternion[3] = sub_imu_orie_local[2];
}