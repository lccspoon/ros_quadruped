#ifndef COMPUTE_WBC_H
#define COMPUTE_WBC_H

#include "common/mathTypes.h"
#include "common/unitreeRobot.h"
#include "common/hexpodRobot.h"
#include <iostream>
#include <eigen3/Eigen/Core>

// // 引入 Pinocchio 中用于多体系统和 FCL（快速碰撞检测库）交互的头文件
// #include "pinocchio/multibody/fcl.hpp"
// // 引入 Pinocchio 中用于解析 URDF（Unified Robot Description Format）文件的头文件
// #include "pinocchio/parsers/urdf.hpp"

// // 引入 Pinocchio 中处理关节配置相关算法的头文件
// #include "pinocchio/algorithm/joint-configuration.hpp"
// // 引入 Pinocchio 中进行正向运动学计算的头文件
// #include "pinocchio/algorithm/kinematics.hpp"
// 引入 Pinocchio 中处理几何模型相关算法的头文件
// #include "pinocchio/algorithm/geometry.hpp"

class NullSpaceOp_wbc
{
    public:
        NullSpaceOp_wbc(/* args */);
        
        Eigen::Matrix<double, 24, 1> q_cmd;
        Eigen::Matrix<double, 24, 1> qd_cmd;
        Eigen::Matrix<double, 24, 1> qdd_cmd;

    // private:
        /*假设 Pinocchio 已计算得到的变量（已知量）*/
        Eigen::Matrix<double, 24, 24> M; // 质量矩阵 24x24
        Eigen::Matrix<double, 24, 1> h; // 非线性项（重力、科氏力等） 24x1

        //task1
        Eigen::Matrix<double, 9, 24> J_support;  // 支撑腿雅可比矩阵（假设 3 条支撑腿，每条腿 3 DOF） 9x24
        //task2
        Eigen::Matrix<double, 3, 24> J_body_rot;  // 机身旋转雅可比矩阵 3x24
        //task3
        Eigen::Matrix<double, 3, 24> J_body_trans;  // 机身平动雅可比矩阵 3x24
        //task4
        Eigen::Matrix<double, 9, 24> J_swing;  // 摆动腿雅可比矩阵（假设 3 条摆动腿，每条腿 3 DOF） 9x24

        Eigen::Matrix<double, 24, 1> q;  // 当前关节位置 24x1
        Eigen::Matrix<double, 24, 1> v;  // 当前关节速度 24x1

        Eigen::Matrix<double, 18, 1> mpc_qdd;  // mpc 计算得到的关节加速度（假设已知）
        Eigen::Matrix<double, 18, 1> mpc_f;  // mpc 计算得到的关节力矩（假设已知）

        // Relaxation Optimization 松弛优化变量
        Eigen::Matrix<double,6, 1> delta_qdd;  // 需要求解的量 6x1
        Eigen::Matrix<double, 18, 1> delta_f;  // 需要求解的量  18x1

        /*4个任务级*/
        Eigen::Matrix<double, 9, 1> v_support_des; // 支撑腿期望速度（3 条腿 × 3）
        Eigen::Matrix<double, 3, 1> v_body_rot_des; // 机身旋转期望速度
        Eigen::Matrix<double, 3, 1> v_body_trans_des; // 机身平动期望速度
        Eigen::Matrix<double, 9, 1> v_swing_des; // 摆动腿期望速度（3 条腿 × 3）

        // 零空间优化主方法
        void compute_nullspace_optimization(
            Eigen::Matrix<double, 24, 1>& q_out,  // 输出关节角度
            Eigen::Matrix<double, 24, 1>& v_out,  // 输出关节速度
            Eigen::Matrix<double, 24, 1>& qdd_out // 输出关节加速度
        );

        // 4 个任务的零空间计算方法
        void compute_support_task(
            Eigen::Matrix<double, 24, 1>& qdd,        // 输出加速度贡献
            Eigen::Matrix<double, 24, 24>& N_out,     // 输出更新后的零空间
            const Eigen::Matrix<double, 24, 24>& N_in // 输入前一任务的零空间
        );

        void compute_body_rot_task(
            Eigen::Matrix<double, 24, 1>& qdd,        // 输出加速度贡献
            Eigen::Matrix<double, 24, 24>& N_out,     // 输出更新后的零空间
            const Eigen::Matrix<double, 24, 24>& N_in // 输入前一任务的零空间
        );

        void compute_body_trans_task(
            Eigen::Matrix<double, 24, 1>& qdd,        // 输出加速度贡献
            Eigen::Matrix<double, 24, 24>& N_out,     // 输出更新后的零空间
            const Eigen::Matrix<double, 24, 24>& N_in // 输入前一任务的零空间
        );

        void compute_swing_task(
            Eigen::Matrix<double, 24, 1>& qdd,        // 输出加速度贡献
            Eigen::Matrix<double, 24, 24>& N_out,     // 输出更新后的零空间
            const Eigen::Matrix<double, 24, 24>& N_in // 输入前一任务的零空间
        );

        // 手动计算伪逆的函数
        template <int Rows, int Cols>
        Eigen::Matrix<double, Cols, Rows> compute_pseudo_inverse(
            const Eigen::Matrix<double, Rows, Cols>& A
        );

        void compute_nullspace_optimization_lcc( 
            Eigen::Matrix<double, 24, 1> q, 
            Eigen::Matrix<double, 24, 1> qd, 
        
            Eigen::Matrix<double, 9, 24> J_1,
        
            Eigen::Matrix<double, 3, 1> x_2, 
            Eigen::Matrix<double, 3, 1> x_2_d,
            Eigen::Matrix<double, 3, 1> xd_2,
            Eigen::Matrix<double, 3, 1> xd_2_d,
            Eigen::Matrix<double, 3, 1> xdd_2_d,
            Eigen::Matrix<double, 3, 24> J_2,
            Eigen::Matrix<double, 3, 24> Jd_2,
        
            Eigen::Matrix<double, 3, 1> x_3, 
            Eigen::Matrix<double, 3, 1> x_3_d,
            Eigen::Matrix<double, 3, 1> xd_3,
            Eigen::Matrix<double, 3, 1> xd_3_d,
            Eigen::Matrix<double, 3, 1> xdd_3_d,
            Eigen::Matrix<double, 3, 24> J_3,
            Eigen::Matrix<double, 3, 24> Jd_3,
        
            Eigen::Matrix<double, 9, 1> x_4, 
            Eigen::Matrix<double, 9, 1> x_4_d,
            Eigen::Matrix<double, 9, 1> xd_4,
            Eigen::Matrix<double, 9, 1> xd_4_d,
            Eigen::Matrix<double, 9, 1> xdd_4_d,
            Eigen::Matrix<double, 9, 24> J_4,
            Eigen::Matrix<double, 9, 24> Jd_4
        
        );

        void compute_nullspace_optimization_lcc_test(void);

};






#endif