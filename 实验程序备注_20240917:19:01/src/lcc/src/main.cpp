/**********************************************************************
 @author lcc
 @date 20240412
 @cite unitree_guide 
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#include "interface/IOSDK.h"

#include "control/OsqpMpcTest.h"

#include <thread> 
#include <chrono> 

#include "control/OsqpMpcTest.h"
#include "interface/imu.h"

#if USE_A_REAL_HEXAPOD == false
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
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

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig){
    std::cout << "lcc stop the controller" << std::endl;
    running = false;
    exit(0);
}

void setProcessScheduler(){
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1){
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv){
    setProcessScheduler();  //如果线程启动失败，那么可以启用超级权限
    std::cout << std::fixed << std::setprecision(4);

    // #ifdef RUN_ROS
    ros::init(argc, argv, "lcc2423f");
    // #endif // RUN_ROS

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
    
    #if USE_A_REAL_HEXAPOD == true
        ioInter = new IOSDK();
        ctrlPlat = CtrlPlatform::REALROBOT;
    #else
        ioInter = new IOROS();
        ctrlPlat = CtrlPlatform::GAZEBO;
    #endif 

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz
    // ctrlComp->dt = 0.00225; // run 
    // ctrlComp->dt = 0.0025; // lcc
    // ctrlComp->dt = 0.003; // lcc
    ctrlComp->running = &running;

    // #if IS_THIS_A_HEXAPOD
        ctrlComp->sixlegdogModel = new SixLegDogRobot();
    // #else
        // ctrlComp->robotModel = new A1Robot();
        // ctrlComp->robotModel = new Go1Robot();
    // #endif

    Vec6 _bias;
    _bias << 0, 0.5, 0.5, 0, 0, 0.5;

    #if USE_A_REAL_HEXAPOD == false
    // ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, _bias); // Trot
    ctrlComp->waveGen = new WaveGenerator(0.55, 0.5, _bias); // Trot
    // ctrlComp->waveGen = new WaveGenerator(0.6, 0.5, _bias); // Trot
    // ctrlComp->waveGen = new WaveGenerator(0.8, 0.5, _bias); // Trot
    #else
    // ctrlComp->waveGen = new WaveGenerator(1, 0.5, _bias); // Trotss
    ctrlComp->waveGen = new WaveGenerator(0.75, 0.5, _bias); // Trot
    #endif

    ctrlComp->geneObj();
    ControlFrame ctrlFrame(ctrlComp);
    signal(SIGINT, ShutDown);

    #if USE_A_REAL_HEXAPOD == false
        RosTopicMsgPub COM("COM");
        RosTopicMsgPub VEL("VEL");
        RosTopicMsgPub RPY("RPY");
        while (running){   

            // if( ctrlFrame.ctrlComp->lowState->userFunctionMode.motor_disenable_flag == 1 ){
            //     spi.exit_close_loop();
            //     printf("\n ininininin \n");
            // }
            // else{

            // }
            // // printf("\n ininininin \n");
            // // std::cout<<"motor_disenable_flag:  "<< ctrlFrame.ctrlComp->lowState->userFunctionMode.motor_disenable_flag <<std::endl;
            // // std::cout<<"function_test:  "<< ctrlFrame.ctrlComp->lowState->userFunctionMode.function_test <<std::endl;
            // usleep(2000);

            // auto t1 = std::chrono::high_resolution_clock::now();
            // printf("\n-a-gf-asg-\n");
            ctrlFrame.run();
            // COM.msgPubRun( ctrlComp->estimator->getPosition() );
            // VEL.msgPubRun( ctrlComp->estimator->getVelocity() );
            // RPY.msgPubRun( rotMatToRPY(ctrlComp->lowState->getRotMat()));
            // 延时2毫秒
            // std::this_thread::sleep_for(std::chrono::milliseconds(2));

            // auto t2 = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            // std::cout << "ctrlFrame.run() in " << ms_double.count() << "ms" << std::endl;
        }
    #else
        std::atomic<bool> control_execute{};
        control_execute.store(true, std::memory_order_release);

        std::thread spi_can_run([&]() {
            Vec36 init_motor_set_q;
            Vec36 dou_dong_angle;
            bool get_motor_response_flag = false;
            long long startTime;
            while (control_execute.load(std::memory_order_acquire)  && running) {

            //     if( MOTOR_DISABEL_FLAG == true ){ // 'o'
            //         spi_2.exit_close_loop();
            //         init_motor_set_q = ctrlComp->lowState->getQ_Hex();

            //         Vec36 motor_set;
            //         Vec36 motor_kp;
            //         Vec36 motor_kd;
            //         // MTX_MOTORCMD_2.lock();
            //         for (int i = 0; i < 18; i++)
            //         {
            //             ctrlComp->lowCmd->motorCmd[i].q = init_motor_set_q(i);
            //             motor_set(i) = ctrlComp->lowCmd->motorCmd[i].q;
            //             ctrlComp->lowCmd->setAllLegGain(30, 1);
            //         }

            //         if( ctrlComp->lowCmd->motorCmd[0].q == 0.000 or ctrlComp->lowCmd->motorCmd[1].q == 0.000 or ctrlComp->lowCmd->motorCmd[2].q == 0.000 or
            //         ctrlComp->lowCmd->motorCmd[3].q == 0.000 or ctrlComp->lowCmd->motorCmd[4].q == 0.000 or ctrlComp->lowCmd->motorCmd[5].q == 0.000 or
            //         ctrlComp->lowCmd->motorCmd[6].q == 0.000 or ctrlComp->lowCmd->motorCmd[7].q == 0.000 or ctrlComp->lowCmd->motorCmd[8].q == 0.000 or
            //         ctrlComp->lowCmd->motorCmd[9].q == 0.000 or ctrlComp->lowCmd->motorCmd[10].q == 0.000 or ctrlComp->lowCmd->motorCmd[11].q == 0.000 or
            //         ctrlComp->lowCmd->motorCmd[12].q == 0.000 or ctrlComp->lowCmd->motorCmd[13].q == 0.000 or ctrlComp->lowCmd->motorCmd[14].q == 0.000 or
            //         ctrlComp->lowCmd->motorCmd[15].q == 0.000 or ctrlComp->lowCmd->motorCmd[16].q == 0.000 or ctrlComp->lowCmd->motorCmd[17].q == 0.000 )
            //             get_motor_response_flag = false;
            //         else
            //             get_motor_response_flag = true;

            //         // MTX_MOTORCMD_2.unlock();

            //         MOTOR_ENTER_CLOSELOOP = false;
            //         MOTOR_READY_FLAG = false;
            //         MOTOR_ENABLE_FLAG = false;
            //         // std::cout<<"\n MOTOR_DISABEL_FLAG : rec_moter_q:  \n"<< spi_2.rec_moter_q* 180/3.1415926 <<std::endl;
            //         // std::cout<<"\n MOTOR_DISABEL_FLAG : getQ_Hex:  \n"<< ctrlComp->lowState->getQ_Hex() * 180/3.1415926 <<std::endl;
            //         std::cout<<"MOTOR_DISABEL_FLAG : motor_set:  \n"<< motor_set * 180/3.1415926 <<std::endl;
            //         // printf(" _yaw: %f \n", ctrlComp->lowState->getYaw()*180/3.1415926);
            //         std::cout<<"rotMatToRPY:"<< rotMatToRPY(ctrlComp->lowState->getRotMat()).transpose()*180/3.1415926 <<std::endl;
            //     }
            //     else if( MOTOR_ENABLE_FLAG == true && get_motor_response_flag == true){ // 'p'
            //         for (int i = 0; i < 6; i++){
            //             spi_2.enter_close_loop();
            //             usleep(5000);
            //         }
            //         init_motor_set_q = ctrlComp->lowState->getQ_Hex();
            //         MOTOR_ENABLE_FLAG = false;
            //         MOTOR_ENTER_CLOSELOOP = true;
            //     }
            //     else if( MOTOR_READY_FLAG == true && MOTOR_ENTER_CLOSELOOP == true){ // '['
            //         spi_2.send_all_data();
            //         dou_dong_angle.setZero();
            //         for (int i = 0; i < 18; i++){
            //             dou_dong_angle(i) = DOU_DONG_ANGEL;
            //         }
            //         ctrlComp->lowCmd->setQ( vec36ToVec18( init_motor_set_q + dou_dong_angle )  );
            //     }
            //     else{
            //     }

            //     startTime = getSystemTime();

            //     if( MOTOR_DATA_LOAD == true && MOTOR_ENTER_CLOSELOOP == true){ // ']'
            //         spi_2.send_all_data();
            //     }
            //     else{
            //         // printf(" _yaw: %f \n", ctrlComp->lowState->getYaw()*180/3.1415926);
            //     }
            //     absoluteWait(startTime, (long long)(ctrlComp->dt * 1000000));
            }
        });

        std::thread imu_recv([&]() {
            long long startTime;
            while (control_execute.load(std::memory_order_acquire)  && running) {
                // startTime = getSystemTime();
                imu_run();
                absoluteWait(startTime, (long long)(ctrlComp->dt * 1000000));
            }
        });

        std::thread main_thread([&]() {
            while (control_execute.load(std::memory_order_acquire) && running) {
                ctrlFrame.run();
            }
        });


        std::thread save_data2txt([&]() {

            #if TXT_FLAGE
            ofstream CoT_motor_t;
            ofstream RPY_act_des_slopDegree;
            ofstream CoG_PosActDes_VelActDes;
            ofstream foot_hip_z;
            ofstream foot_forceZ_forceSum;
            CoT_motor_t.open("./CoT_motor_t.txt", ios::out | ios::trunc);
            RPY_act_des_slopDegree.open("./RPY_act_des_slopDegree.txt", ios::out | ios::trunc);
            CoG_PosActDes_VelActDes.open("./CoG_PosActDes_VelActDes.txt", ios::out | ios::trunc);
            foot_hip_z.open("./foot_hip_z.txt", ios::out | ios::trunc);
            foot_forceZ_forceSum.open("./foot_forceZ_forceSum.txt", ios::out | ios::trunc);
            #endif

            while (control_execute.load(std::memory_order_acquire) && running) {
            usleep(10000);
            #if TXT_FLAGE
            if( DTAT_SAVE2TXT == true ){


                if ( ! CoT_motor_t) { cout << "CoT_motor_t 文件不能打开" <<endl; }
                    CoT_motor_t <<  
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[0].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[1].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[2].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[3].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[4].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[5].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[6].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[7].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[8].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[9].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[10].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[11].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[12].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[13].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[14].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[15].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[16].tauEst <<" "<< 
                    ctrlFrame._FSMController->_ctrlComp->lowState->motorState[17].tauEst << "\n";

                Vec3 RPY;    
                RPY = rotMatToRPY(ctrlFrame._FSMController->_ctrlComp->lowState->getRotMat());
                if ( ! RPY_act_des_slopDegree) { cout << "RPY_act_des_slopDegree 文件不能打开" <<endl; }
                    RPY_act_des_slopDegree <<  
                    RPY(0) <<" "<< 
                    RPY(1) <<" "<< 
                    RPY(2) <<" "<< 
                    RPY_DES(0) <<" "<< 
                    RPY_DES(1) <<" "<< 
                    RPY_DES(2) <<" "<< 
                    TERRIAN_EST_DEGREE(0) <<" "<< 
                    TERRIAN_EST_DEGREE(1) << "\n";

                Vec3 vel, pos;    
                pos = ctrlFrame._FSMController->_ctrlComp->estimator->getPosition();
                vel = ctrlFrame._FSMController->_ctrlComp->estimator->getVelocity();
                if ( ! CoG_PosActDes_VelActDes) { cout << "CoG_PosActDes_VelActDes 文件不能打开" <<endl; }
                    CoG_PosActDes_VelActDes <<  
                    pos(0) <<" "<< 
                    pos(1) <<" "<< 
                    pos(2) <<" "<< 
                    POS_WORLD_DES(0) <<" "<< 
                    POS_WORLD_DES(1) <<" "<< 
                    POS_WORLD_DES(2) <<" "<< 
                    vel(0) <<" "<< 
                    vel(1) <<" "<< 
                    vel(2) <<" "<< 
                    VEL_WORLD_DES(0) <<" "<< 
                    VEL_WORLD_DES(1) <<" "<< 
                    VEL_WORLD_DES(2) << "\n";

                Vec36 foot_hip_pos;
                foot_hip_pos = ctrlFrame._FSMController->_ctrlComp->sixlegdogModel->getFeet2BPositions(
                    *(ctrlFrame._FSMController->_ctrlComp->lowState),
                    FrameType::BODY );
                if ( ! foot_hip_z) { cout << "foot_hip_z 文件不能打开" <<endl; }
                    foot_hip_z <<  
                    foot_hip_pos(2) <<" "<< 
                    foot_hip_pos(5) <<" "<< 
                    foot_hip_pos(8) <<" "<< 
                    foot_hip_pos(11) <<" "<< 
                    foot_hip_pos(14) <<" "<< 
                    foot_hip_pos(17) << "\n";
                    
                Vec36 _footTipForceEst;
                _footTipForceEst = ctrlFrame._FSMController->_ctrlComp->sixlegdogModel->calcForceByTauEst( 
                    vec36ToVec18(
                        ctrlFrame._FSMController->_ctrlComp->lowState->getQ_Hex()), 
                        ctrlFrame._FSMController->_ctrlComp->lowState->getTau_Hex()
                        );

                Vec6 forceSum;
                forceSum(0) = sqrt(pow(_footTipForceEst(0), 2) + pow(_footTipForceEst(1), 2) + pow(_footTipForceEst(2), 2) ); 
                forceSum(1) = sqrt(pow(_footTipForceEst(3), 2) + pow(_footTipForceEst(4), 2) + pow(_footTipForceEst(5), 2) ); 
                forceSum(2) = sqrt(pow(_footTipForceEst(6), 2) + pow(_footTipForceEst(7), 2) + pow(_footTipForceEst(8), 2) ); 
                forceSum(3) = sqrt(pow(_footTipForceEst(9), 2) + pow(_footTipForceEst(10), 2) + pow(_footTipForceEst(11), 2) ); 
                forceSum(4) = sqrt(pow(_footTipForceEst(12), 2) + pow(_footTipForceEst(13), 2) + pow(_footTipForceEst(14), 2) ); 
                forceSum(5) = sqrt(pow(_footTipForceEst(15), 2) + pow(_footTipForceEst(16), 2) + pow(_footTipForceEst(17), 2) ); 

                if ( ! foot_forceZ_forceSum) { cout << "foot_forceZ_forceSum 文件不能打开" <<endl; }
                    foot_forceZ_forceSum <<  
                    _footTipForceEst(2) <<" "<< 
                    _footTipForceEst(5) <<" "<< 
                    _footTipForceEst(8) <<" "<< 
                    _footTipForceEst(11) <<" "<< 
                    _footTipForceEst(14) <<" "<< 
                    _footTipForceEst(17) <<" "<< 
                    forceSum(0) <<" "<< 
                    forceSum(1) <<" "<< 
                    forceSum(2) <<" "<< 
                    forceSum(3) <<" "<< 
                    forceSum(4) <<" "<< 
                    forceSum(5) << "\n";
            }
            #endif
            }
        });

        // spi_can_run.join();
        imu_recv.join();
        main_thread.join();
        #if TXT_FLAGE
        save_data2txt.join();
        #endif
    #endif

    return 0;
}