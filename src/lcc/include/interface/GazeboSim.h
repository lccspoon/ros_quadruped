#ifndef _GAZEBOSIM_H
#define _GAZEBOSIM_H

// stl
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>


// std
#include <Eigen/Dense>
#include <memory>
#include "math.h"
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// ROS
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
#include "std_msgs/Float64MultiArray.h"

#include <eigen3/Eigen/Core>
#include <Eigen/Dense>


#define NUM_LEG 4
#define LEG_DOF 3
#define NUM_DOF 12
#define NUM_MOD 3 // 运动模式数量

class GazeboSim
{
public:
    // GazeboSim(int argc, char *argv[]);
    GazeboSim();

    ~GazeboSim();

    void simJointForcePub(Eigen::Matrix<double, LEG_DOF, NUM_LEG> pub_msg);

    Eigen::Matrix<double, LEG_DOF, NUM_LEG> retSimJointP();

    Eigen::Matrix<double, LEG_DOF, NUM_LEG> retSimJointV();

    Eigen::Matrix<double, LEG_DOF, NUM_LEG> retSimJointT();

    Eigen::Matrix<double, 4, 1> retSimImuQuat();

    Eigen::Matrix<double, 3, 1> retSimImuAngV();
    
    Eigen::Matrix<double, 3, 1> retSimImuLinA();

    Eigen::Matrix<double, 3, 1> retSimOdeBodyP();

    Eigen::Matrix<double, 3, 1> retSimOdeBodyV();

private:
    //  0~12: RF LF RB LB 
    ros::Publisher pub_joint_cmd[NUM_DOF];    
    std_msgs::Float64 pub_joint_msg[NUM_DOF];

    ros::Subscriber sub_joint_states;
    ros::Subscriber sub_imu_msg;
    ros::Subscriber sub_nav_odometry;
};

class RosTopicMsgPub
{
protected:
    ros::Publisher msgPub;
    std_msgs::Float64 msgTemp;
    std_msgs::Float64MultiArray msgTempArray;
    ros::NodeHandle node;
public:
    RosTopicMsgPub(std::string topicName) {
        msgPub=node.advertise<std_msgs::Float64MultiArray>(topicName,1000);
        // std::cout<<topicName<<std::endl;
    }
    void msgPubRun(double * msg_array){
        msgTempArray.data = {msg_array[0], msg_array[1], msg_array[2]};
        msgPub.publish(msgTempArray);
    }
    void msgPubRun(Eigen::Matrix<double,3,1> msg_matrix){
        msgTempArray.data = {msg_matrix(0), msg_matrix(1), msg_matrix(2)};
        msgPub.publish(msgTempArray);
    }
    void msgPubRun(Eigen::Matrix<double,4,1> msg_matrix){
        msgTempArray.data = {msg_matrix(0), msg_matrix(1), msg_matrix(2), msg_matrix(3)};
        msgPub.publish(msgTempArray); 
    }
    void msgPubRun(Eigen::Matrix<double,6,1> msg_matrix) {
        msgTempArray.data = {msg_matrix(0), msg_matrix(1), msg_matrix(2), msg_matrix(3), msg_matrix(4), msg_matrix(5)};
        msgPub.publish(msgTempArray);
    }
    void msgPubRun(Eigen::Matrix<double,18,1> msg_matrix) {
        msgTempArray.data={
            msg_matrix(0),msg_matrix(1),msg_matrix(2),msg_matrix(3),msg_matrix(4),msg_matrix(5),
            msg_matrix(6),msg_matrix(7),msg_matrix(8),msg_matrix(9),msg_matrix(10),msg_matrix(11),
            msg_matrix(12),msg_matrix(13),msg_matrix(14),msg_matrix(15),msg_matrix(16),msg_matrix(17),
            };
        msgPub.publish(msgTempArray);
    }

};




#endif