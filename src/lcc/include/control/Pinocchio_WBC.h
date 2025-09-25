#ifndef PINOCCHIO_WBC_H
#define PINOCCHIO_WBC_H

#include <iostream>
// #include <vector>
// #include <Eigen/Dense>
// #include "pinocchio/multibody/fcl.hpp"
// #include "pinocchio/parsers/urdf.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/geometry.hpp"
// #include "pinocchio/algorithm/jacobian.hpp"
// #include "pinocchio/algorithm/crba.hpp"
// #include "pinocchio/algorithm/rnea.hpp"
// #include "pinocchio/algorithm/frames.hpp"
// #include "pinocchio/algorithm/center-of-mass.hpp" // 确保质心计算

// const std::string URDF_PATH = "/home/lcc/1_noetic_ws/20250329_ws/src/hexapod_v3/urdf/hexapod_v3.urdf";
// const std::string MESH_DIR = "/home/lcc/1_noetic_ws/20250329_ws/src/hexapod_v3/urdf";

// class Pinocchio_WBC {
// public:
//     Pinocchio_WBC();
    
//     void pinocchio_test();

//     void compute_wbc_matrices(
//         const Eigen::VectorXd& q,           // 关节位置 (24x1)
//         const Eigen::VectorXd& v,           // 关节速度 (24x1)
//         const std::vector<int>& support_legs, // 支撑腿编号
//         const std::vector<int>& swing_legs,   // 摆动腿编号
//         Eigen::MatrixXd& J_1,               // 支撑腿雅可比 (9x24)
//         Eigen::MatrixXd& Jd_2,              // 身体任务雅可比导数 (6x24)
//         Eigen::MatrixXd& Jd_3,              // 质心任务雅可比导数 (6x24)
//         Eigen::MatrixXd& J_4,               // 摆动腿雅可比 (9x24)
//         Eigen::MatrixXd& Jd_4,              // 摆动腿雅可比导数 (9x24)
//         Eigen::MatrixXd& M,                 // 质量矩阵 (24x24)
//         Eigen::VectorXd& C,                 // 非线性项 (24x1)
//         Eigen::MatrixXd& Jc                 // 接触雅可比 (9x24)
//     );

//     void test_wbc_matrices(); // 测试函数

// private:
//     pinocchio::Model model_;
//     pinocchio::Data data_;
//     pinocchio::GeometryModel collision_model_;
//     pinocchio::GeometryData collision_data_;
//     pinocchio::GeometryModel visual_model_;
//     pinocchio::GeometryData visual_data_;
// };

#endif