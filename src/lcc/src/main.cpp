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

// #ifdef COMPILE_WITH_REAL_ROBOT
// #include "interface/IOSDK.h"
// #endif // COMPILE_WITH_REAL_ROBOT

// #ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
// #endif // COMPILE_WITH_ROS

#include "control/OsqpMpcTest.h"//lcc

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::milliseconds

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig){
    std::cout << "lcc stop the controller" << std::endl;
    running = false;
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
    /* set real-time process */
    setProcessScheduler();  //如果线程启动失败，那么可以启用超级权限
    /* set the print format */
    // 这段代码用于设置 `std::cout` 的输出格式。
    // 具体来说，`std::fixed` 表示浮点数将以固定小数位数的格式输出，而 `std::setprecision(3)` 则表示设置输出浮点数的小数位数为 3。
    // 因此，该代码段会导致 `std::cout` 输出的浮点数保留 3 位小数。
    std::cout << std::fixed << std::setprecision(3);

    // osqpMpcTest();//lcc
    // printf("\naaa\n");

// #ifdef RUN_ROS
    ros::init(argc, argv, "lcc2423f");
// #endif // RUN_ROS

    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
    
// #ifdef COMPILE_WITH_SIMULATION
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
// #endif // COMPILE_WITH_SIMULATION

// // #ifdef COMPILE_WITH_REAL_ROBOT
// //     ioInter = new IOSDK();
// //     ctrlPlat = CtrlPlatform::REALROBOT;
// // #endif // COMPILE_WITH_REAL_ROBOT

    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.002; // run at 500hz
    // // ctrlComp->dt = 0.0025; // lcc
    ctrlComp->running = &running;

//     #ifdef ROBOT_TYPE_A1
        ctrlComp->robotModel = new A1Robot();
//     #endif
//     #ifdef ROBOT_TYPE_Go1
//         ctrlComp->robotModel = new Go1Robot();
//     #endif

    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
//     // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
//     // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
//     // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
//     // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    ctrlComp->geneObj();

    ControlFrame ctrlFrame(ctrlComp);

    // 由于 KeyBoard 类继承自 CmdPanel 类，因此 KeyBoard 对象也被视为一种 CmdPanel 对象。
    // 这就是继承的基本概念，子类对象可以赋值给父类指针或引用。
    // cmdPanel = new KeyBoard();

    // KeyBoard keyb;

    // IOROS_lcc as;

    signal(SIGINT, ShutDown);
    // printf(" \n  aaaafaf \n ");

    // //! 12 在你的代码中表示你创建的异步微调对象将同时处理多达 12 个回调函数。这个数字可以根据你的应用程序的性能需求进行调整，以获得最佳性能。
    // ros::AsyncSpinner spinner(3);
    // spinner.start();

    while (running)
    {   
        auto t1 = std::chrono::high_resolution_clock::now();

        ctrlFrame.run();

        // 延时2毫秒
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));

        auto t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms_double = t2 - t1;
        // std::cout << "ctrlFrame.run() in " << ms_double.count() << "ms" << std::endl;
    }

    // delete ctrlComp;
    return 0;
}


