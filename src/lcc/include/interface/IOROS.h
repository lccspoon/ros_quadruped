/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
// #ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "interface/GazeboSim.h"
// #include "unitree_legged_msgs/LowCmd.h"
// #include "unitree_legged_msgs/LowState.h"
// #include "unitree_legged_msgs/MotorCmd.h"
// #include "unitree_legged_msgs/MotorState.h"
// #include <sensor_msgs/Imu.h>
#include <string>

#define NUM_LEG 4
#define LEG_DOF 3
#define NUM_DOF 12
#define NUM_MOD 3 // 运动模式数量

// class IOROS : public IOInterface, public GazeboSim{
class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);





//  0~12: RF LF RB LB 
ros::Publisher pub_joint_cmd[NUM_DOF];    
std_msgs::Float64 pub_joint_msg[NUM_DOF];

ros::Subscriber sub_joint_states;
ros::Subscriber sub_imu_msg;
ros::Subscriber sub_nav_odometry;

// GazeboSim gs;

// ros::NodeHandle _nm;
// ros::Subscriber _servo_sub[12], _imu_sub;
// ros::Publisher _servo_pub[12];
// unitree_legged_msgs::LowCmd _lowCmd;
// unitree_legged_msgs::LowState _lowState;
// std::string _robot_name;

//repeated functions for multi-thread
ros::NodeHandle nh;
void initRecv();
void initSend();

// void simJointForcePub(Eigen::Matrix<double, LEG_DOF, NUM_LEG> pub_msg);

// Eigen::Matrix<double, LEG_DOF, NUM_LEG> retSimJointP();

// Eigen::Matrix<double, LEG_DOF, NUM_LEG> retSimJointV();

// Eigen::Matrix<double, LEG_DOF, NUM_LEG> retSimJointT();

// Eigen::Matrix<double, 4, 1> retSimImuQuat();

// Eigen::Matrix<double, 3, 1> retSimImuAngV();

// Eigen::Matrix<double, 3, 1> retSimImuLinA();

// Eigen::Matrix<double, 3, 1> retSimOdeBodyP();

// Eigen::Matrix<double, 3, 1> retSimOdeBodyV();

// //Callback functions for ROS
// void imuCallback(const sensor_msgs::Imu & msg);

// void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
// void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
// void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

// void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
// void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
// void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

// void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
// void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
// void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

// void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
// void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
// void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H

// #endif  // COMPILE_WITH_ROS