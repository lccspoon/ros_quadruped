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
    // ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->dt = 0.0025; // lcc
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
    // ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, _bias); // Trot
    ctrlComp->waveGen = new WaveGenerator(0.55, 0.5, _bias); // Trot
    // ctrlComp->waveGen = new WaveGenerator(0.8, 0.5, _bias); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1, 0.5, _bias); // Trot
    // ctrlComp->waveGen = new WaveGenerator(3, 0.5, _bias); // Trot

    ctrlComp->geneObj();
    ControlFrame ctrlFrame(ctrlComp);
    signal(SIGINT, ShutDown);

    #if USE_A_REAL_HEXAPOD == false
        RosTopicMsgPub COM("COM");
        RosTopicMsgPub VEL("VEL");
        RosTopicMsgPub RPY("RPY");
        while (running){   

            // if( ctrlFrame._ctrlComp->lowState->userFunctionMode.motor_disenable_flag == 1 ){
            //     spi.exit_close_loop();
            //     printf("\n ininininin \n");
            // }
            // else{

            // }
            // // printf("\n ininininin \n");
            // // std::cout<<"motor_disenable_flag:  "<< ctrlFrame._ctrlComp->lowState->userFunctionMode.motor_disenable_flag <<std::endl;
            // // std::cout<<"function_test:  "<< ctrlFrame._ctrlComp->lowState->userFunctionMode.function_test <<std::endl;
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

        // std::thread compute_foot_forces_grf_thread([&]() {
        //     while (control_execute.load(std::memory_order_acquire)  && running) {
        //         ;
        //     }
        // });

        std::thread imu_recv([&]() {
            while (control_execute.load(std::memory_order_acquire)  && running) {
                imu_run();
            }
        });

        std::thread main_thread([&]() {
            while (control_execute.load(std::memory_order_acquire) && running) {
                ctrlFrame.run();
            }
        });

        // compute_foot_forces_grf_thread.join();
        imu_recv.join();
        main_thread.join();
    #endif

    return 0;
}


