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
    pub_joint_cmd[0] = nh.advertise<std_msgs::Float64>("/quadruped/RfJ1_jointcc/command", 1000);
    pub_joint_cmd[1] = nh.advertise<std_msgs::Float64>("/quadruped/RfJ2_jointcc/command", 1000);
    pub_joint_cmd[2] = nh.advertise<std_msgs::Float64>("/quadruped/RfJ3_jointcc/command", 1000);

    pub_joint_cmd[3] = nh.advertise<std_msgs::Float64>("/quadruped/LfJ1_jointcc/command", 1000);
    pub_joint_cmd[4] = nh.advertise<std_msgs::Float64>("/quadruped/LfJ2_jointcc/command", 1000);
    pub_joint_cmd[5] = nh.advertise<std_msgs::Float64>("/quadruped/LfJ3_jointcc/command", 1000);

    pub_joint_cmd[6] = nh.advertise<std_msgs::Float64>("/quadruped/RbJ1_jointcc/command", 1000);
    pub_joint_cmd[7] = nh.advertise<std_msgs::Float64>("/quadruped/RbJ2_jointcc/command", 1000);
    pub_joint_cmd[8] = nh.advertise<std_msgs::Float64>("/quadruped/RbJ3_jointcc/command", 1000);

    pub_joint_cmd[9] = nh.advertise<std_msgs::Float64>("/quadruped/LbJ1_jointcc/command", 1000);
    pub_joint_cmd[10] = nh.advertise<std_msgs::Float64>("/quadruped/LbJ2_jointcc/command", 1000);
    pub_joint_cmd[11] = nh.advertise<std_msgs::Float64>("/quadruped/LbJ3_jointcc/command", 1000);
    printf(" IOROS ros inti.\n");
}

// void IOROS::doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
Eigen::Matrix<double, LEG_DOF, NUM_LEG> sub_joint_p_local, sub_joint_p_local_temp;
Eigen::Matrix<double, LEG_DOF, NUM_LEG> sub_joint_v_local, sub_joint_v_local_temp;
Eigen::Matrix<double, LEG_DOF, NUM_LEG> sub_joint_t_local, sub_joint_t_local_temp;
int sub_joint_state_seq_local;
void doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
{
    sub_joint_state_seq_local = sub_joint_states->header.seq;
    for (int i = 0; i < NUM_DOF; i++)
    {
        sub_joint_p_local_temp(i) = sub_joint_states->position[i];
        sub_joint_v_local_temp(i) = sub_joint_states->velocity[i];
        sub_joint_t_local_temp(i) = sub_joint_states->effort[i];
    }
    //  0~12: RF LF RB LB 
    sub_joint_p_local.block<3,1>(0,0) = sub_joint_p_local_temp.block<3,1>(0,3);
    sub_joint_p_local.block<3,1>(0,1) = sub_joint_p_local_temp.block<3,1>(0,1);
    sub_joint_p_local.block<3,1>(0,2) = sub_joint_p_local_temp.block<3,1>(0,2);
    sub_joint_p_local.block<3,1>(0,3) = sub_joint_p_local_temp.block<3,1>(0,0);

    sub_joint_v_local.block<3,1>(0,0) = sub_joint_v_local_temp.block<3,1>(0,3);
    sub_joint_v_local.block<3,1>(0,1) = sub_joint_v_local_temp.block<3,1>(0,1);
    sub_joint_v_local.block<3,1>(0,2) = sub_joint_v_local_temp.block<3,1>(0,2);
    sub_joint_v_local.block<3,1>(0,3) = sub_joint_v_local_temp.block<3,1>(0,0);

    sub_joint_t_local.block<3,1>(0,0) = sub_joint_t_local_temp.block<3,1>(0,3);
    sub_joint_t_local.block<3,1>(0,1) = sub_joint_t_local_temp.block<3,1>(0,1);
    sub_joint_t_local.block<3,1>(0,2) = sub_joint_t_local_temp.block<3,1>(0,2);
    sub_joint_t_local.block<3,1>(0,3) = sub_joint_t_local_temp.block<3,1>(0,0);

    sub_joint_p_local(1, 1) = -sub_joint_p_local(1, 1);
    sub_joint_p_local(1, 3) = -sub_joint_p_local(1, 3);
    sub_joint_p_local(0, 3) = -sub_joint_p_local(0, 3);
    sub_joint_p_local(2, 0) = -sub_joint_p_local(2, 0);
    sub_joint_p_local(2, 2) = -sub_joint_p_local(2, 2);
    sub_joint_p_local(0, 2) = -sub_joint_p_local(0, 2);

    sub_joint_t_local(1, 1) = -sub_joint_t_local(1, 1);
    sub_joint_t_local(1, 3) = -sub_joint_t_local(1, 3);
    sub_joint_t_local(0, 3) = -sub_joint_t_local(0, 3);
    sub_joint_t_local(2, 0) = -sub_joint_t_local(2, 0);
    sub_joint_t_local(2, 2) = -sub_joint_t_local(2, 2);
    sub_joint_t_local(0, 2) = -sub_joint_t_local(0, 2);

    sub_joint_v_local(1, 1) = -sub_joint_v_local(1, 1);
    sub_joint_v_local(1, 3) = -sub_joint_v_local(1, 3);
    sub_joint_v_local(0, 3) = -sub_joint_v_local(0, 3);
    sub_joint_v_local(2, 0) = -sub_joint_v_local(2, 0);
    sub_joint_v_local(2, 2) = -sub_joint_v_local(2, 2);
    sub_joint_v_local(0, 2) = -sub_joint_v_local(0, 2);

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
    sub_joint_states = nh.subscribe<sensor_msgs::JointState>("/quadruped/joint_states",1,doMsg_yobotics_joint_states);
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
    std_msgs::Float64 pub_data[NUM_DOF];

    Vec12 init_tau;
    init_tau.setZero();
    init_tau(2) = -0.1;
    init_tau(5) = -0.1;
    init_tau(8) = -0.1;
    init_tau(11) = -0.1;

    if (cmdPanel->userFunctionMode.function_test == true )
    {
        for(int m(0); m < NUM_DOF; ++m){
            // pub_data[m].data = lowCmd->motorCmd[m].q;
            pub_data[m].data = init_tau(m);
        }
    }
    else
    {
        for(int m(0); m < NUM_DOF; ++m){
            // pub_data[m].data = lowCmd->motorCmd[m].q;
            pub_data[m].data = lowCmd->motorCmd[m].tau;
        }
    }

    pub_data[2].data = -pub_data[2].data; 
    pub_data[4].data = -pub_data[4].data; 
    pub_data[6].data = -pub_data[6].data; 
    pub_data[8].data = -pub_data[8].data; 
    pub_data[9].data = -pub_data[9].data; 
    pub_data[10].data = -pub_data[10].data; 

    for(int m(0); m < NUM_DOF; ++m){
        // pub_data[m].data = 0;
        pub_joint_cmd[m].publish(pub_data[m]);
    }
    // std::cout << "retSimJointP:\n" <<retSimJointP()<< std::endl;
    ros::spinOnce();
}


void IOROS::recvState(LowlevelState *state){

    for(int i(0); i < 12; ++i){
        state->motorState[i].q = sub_joint_p_local(i);
        state->motorState[i].dq = sub_joint_v_local(i);
        // state->motorState[i].ddq = sub_joint_p_local(i);
        state->motorState[i].tauEst = sub_joint_t_local(i);
    }
    for(int i(0); i < 3; ++i){
        // state->imu.quaternion[i] = sub_imu_orie_local(i);
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