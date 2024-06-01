#include "interface/GazeboSim.h"

// // void GazeboSim::doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
// Eigen::Matrix<double, LEG_DOF, NUM_LEG> sub_joint_p_local, sub_joint_p_local_temp;
// Eigen::Matrix<double, LEG_DOF, NUM_LEG> sub_joint_v_local, sub_joint_v_local_temp;
// Eigen::Matrix<double, LEG_DOF, NUM_LEG> sub_joint_t_local, sub_joint_t_local_temp;
// int sub_joint_state_seq_local;
// void doMsg_yobotics_joint_states(const sensor_msgs::JointState::ConstPtr& sub_joint_states)
// {
//     sub_joint_state_seq_local = sub_joint_states->header.seq;
//     for (int i = 0; i < NUM_DOF; i++)
//     {
//         sub_joint_p_local_temp(i) = sub_joint_states->position[i];
//         sub_joint_v_local_temp(i) = sub_joint_states->velocity[i];
//         sub_joint_t_local_temp(i) = sub_joint_states->effort[i];
//     }

//     //  0~12: RF LF RB LB 
//     sub_joint_p_local.block<3,1>(0,0) = sub_joint_p_local_temp.block<3,1>(0,3);
//     sub_joint_p_local.block<3,1>(0,1) = sub_joint_p_local_temp.block<3,1>(0,1);
//     sub_joint_p_local.block<3,1>(0,2) = sub_joint_p_local_temp.block<3,1>(0,2);
//     sub_joint_p_local.block<3,1>(0,3) = sub_joint_p_local_temp.block<3,1>(0,0);

//     sub_joint_v_local.block<3,1>(0,0) = sub_joint_v_local_temp.block<3,1>(0,3);
//     sub_joint_v_local.block<3,1>(0,1) = sub_joint_v_local_temp.block<3,1>(0,1);
//     sub_joint_v_local.block<3,1>(0,2) = sub_joint_v_local_temp.block<3,1>(0,2);
//     sub_joint_v_local.block<3,1>(0,3) = sub_joint_v_local_temp.block<3,1>(0,0);

//     sub_joint_t_local.block<3,1>(0,0) = sub_joint_t_local_temp.block<3,1>(0,3);
//     sub_joint_t_local.block<3,1>(0,1) = sub_joint_t_local_temp.block<3,1>(0,1);
//     sub_joint_t_local.block<3,1>(0,2) = sub_joint_t_local_temp.block<3,1>(0,2);
//     sub_joint_t_local.block<3,1>(0,3) = sub_joint_t_local_temp.block<3,1>(0,0);
// }

// Eigen::Matrix<double,4,1> sub_imu_orie_local;
// Eigen::Matrix<double,3,1> sub_imu_ang_v_local;
// Eigen::Matrix<double,3,1> sub_imu_lin_a_local;
// int sub_imu_seq_local;
// void doMsg_yobotics_imu_msg(const sensor_msgs::Imu::ConstPtr& sub_imu_msg){
//     sub_imu_seq_local = sub_imu_msg->header.seq;

//     sub_imu_orie_local << sub_imu_msg->orientation.x, sub_imu_msg->orientation.y, sub_imu_msg->orientation.z, sub_imu_msg->orientation.w;
//     sub_imu_ang_v_local << sub_imu_msg->angular_velocity.x, sub_imu_msg->angular_velocity.y, sub_imu_msg->angular_velocity.z;
//     sub_imu_lin_a_local << sub_imu_msg->linear_acceleration.x, sub_imu_msg->linear_acceleration.y, sub_imu_msg->linear_acceleration.z;

//     // std::cout << sub_imu_ang_v_local << std::endl;
// }

// Eigen::Matrix<double,3,1> sub_odo_pos_local;
// Eigen::Matrix<double,3,1> sub_odo_lin_twist_local;
// void doMsg_nav_odometry(const nav_msgs::Odometry::ConstPtr& sub_nav_odometry){
//     sub_odo_pos_local << sub_nav_odometry->pose.pose.position.x, sub_nav_odometry->pose.pose.position.y, 
//                     sub_nav_odometry->pose.pose.position.z;
//     sub_odo_lin_twist_local << sub_nav_odometry->twist.twist.linear.x, sub_nav_odometry->twist.twist.linear.y, 
//                          sub_nav_odometry->twist.twist.linear.z;
//     // std::cout << sub_odo_pos_local << std::endl;
// }

// // GazeboSim::GazeboSim(int argc, char *argv[]){
// GazeboSim::GazeboSim(){
//     // ros::init(argc, argv, "hello");
//     ros::NodeHandle nh;
//     // change ros logger
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//         ros::console::notifyLoggerLevelsChanged();
//     }
//     // make sure the ROS infra DO NOT use sim time, otherwise the controller cannot run with correct time steps
//     std::string use_sim_time;
//     if (ros::param::get("/use_sim_time", use_sim_time)) {
//         if (use_sim_time != "false") {
//             std::cout << "hardware must have real time in order to use this program!" << std::endl;
//             // return -1;
//             exit(0);
//         }
//     }
//     // ROS publisher
//     pub_joint_cmd[0] = nh.advertise<std_msgs::Float64>("/quadruped/RfJ1_jointcc/command", 1000);
//     pub_joint_cmd[1] = nh.advertise<std_msgs::Float64>("/quadruped/RfJ2_jointcc/command", 1000);
//     pub_joint_cmd[2] = nh.advertise<std_msgs::Float64>("/quadruped/RfJ3_jointcc/command", 1000);

//     pub_joint_cmd[3] = nh.advertise<std_msgs::Float64>("/quadruped/LfJ1_jointcc/command", 1000);
//     pub_joint_cmd[4] = nh.advertise<std_msgs::Float64>("/quadruped/LfJ2_jointcc/command", 1000);
//     pub_joint_cmd[5] = nh.advertise<std_msgs::Float64>("/quadruped/LfJ3_jointcc/command", 1000);

//     pub_joint_cmd[6] = nh.advertise<std_msgs::Float64>("/quadruped/RbJ1_jointcc/command", 1000);
//     pub_joint_cmd[7] = nh.advertise<std_msgs::Float64>("/quadruped/RbJ2_jointcc/command", 1000);
//     pub_joint_cmd[8] = nh.advertise<std_msgs::Float64>("/quadruped/RbJ3_jointcc/command", 1000);

//     pub_joint_cmd[9] = nh.advertise<std_msgs::Float64>("/quadruped/LbJ1_jointcc/command", 1000);
//     pub_joint_cmd[10] = nh.advertise<std_msgs::Float64>("/quadruped/LbJ2_jointcc/command", 1000);
//     pub_joint_cmd[11] = nh.advertise<std_msgs::Float64>("/quadruped/LbJ3_jointcc/command", 1000);

//     sub_joint_states = nh.subscribe<sensor_msgs::JointState>("/quadruped/joint_states",2,doMsg_yobotics_joint_states);
//     sub_imu_msg = nh.subscribe<sensor_msgs::Imu>("/imu",2,doMsg_yobotics_imu_msg);
//     sub_nav_odometry = nh.subscribe<nav_msgs::Odometry>("/POS_COM",2,doMsg_nav_odometry);

//     printf(" GazeboSim ros inti.\n");
// }

// GazeboSim::~GazeboSim(){
// }

// void GazeboSim::simJointForcePub(Eigen::Matrix<double, LEG_DOF, NUM_LEG> pub_msg){
//     std_msgs::Float64 pub_data[NUM_DOF];
//     for (int i = 0; i < NUM_DOF; i++){
//         pub_data[i].data = pub_msg(i);
//         pub_joint_cmd[i].publish(pub_data[i]);
//     }
// }

// Eigen::Matrix<double, LEG_DOF, NUM_LEG> GazeboSim::retSimJointP(void){
//     return sub_joint_p_local;
// }

// Eigen::Matrix<double, LEG_DOF, NUM_LEG> GazeboSim::retSimJointV(){
//     return sub_joint_v_local;
// }

// Eigen::Matrix<double, LEG_DOF, NUM_LEG> GazeboSim::retSimJointT(){
//     return sub_joint_t_local;
// }

// Eigen::Matrix<double, 4, 1> GazeboSim::retSimImuQuat(){
//     return sub_imu_orie_local;
// }

// Eigen::Matrix<double, 3, 1> GazeboSim::retSimImuAngV(){
//     return sub_imu_ang_v_local;
// }

// Eigen::Matrix<double, 3, 1> GazeboSim::retSimImuLinA(){
//     return sub_imu_lin_a_local;
// }

// Eigen::Matrix<double, 3, 1> GazeboSim::retSimOdeBodyP(){
//     return sub_odo_pos_local;
// }

// Eigen::Matrix<double, 3, 1> GazeboSim::retSimOdeBodyV(){
//     return sub_odo_lin_twist_local;
// }