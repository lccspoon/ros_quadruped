/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
// #ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "std_msgs/Float64.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PointStamped.h>

// #include "interface/GazeboSim.h"
// #include "unitree_legged_msgs/LowCmd.h"
// #include "unitree_legged_msgs/LowState.h"
// #include "unitree_legged_msgs/MotorCmd.h"
// #include "unitree_legged_msgs/MotorState.h"
// #include <sensor_msgs/Imu.h>
#include <string>
#include "common/unitreeRobot.h"


#define NUM_DOFa 18
extern Vec3 ODE_P;
extern Vec3 ODE_V;

// class IOROS : public IOInterface, public GazeboSim{
class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

Eigen::Matrix<double, 3, 1> retSimOdeBodyP();

Eigen::Matrix<double, 3, 1> retSimOdeBodyV();

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);


//  0~12: RF LF RB LB 
//  0~18: RF LF RM LM RB LB 
ros::Publisher pub_joint_cmd[NUM_DOF_W];    
std_msgs::Float64 pub_joint_msg[NUM_DOF_W];

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


};

#endif  // IOROS_H

// #endif  // COMPILE_WITH_ROS