#include "control/Pinocchio_WBC.h"

// Pinocchio_WBC::Pinocchio_WBC()
//     : data_(model_), collision_data_(collision_model_), visual_data_(visual_model_)
// {
//     try {
//         pinocchio::Model fixed_model;
//         pinocchio::urdf::buildModel(URDF_PATH, fixed_model);
//         pinocchio::JointModelFreeFlyer free_flyer;
//         model_.addJoint(0, free_flyer, pinocchio::SE3::Identity(), "free_flyer");
//         model_.appendBodyToJoint(model_.getJointId("free_flyer"), fixed_model.inertias[fixed_model.getBodyId("base_link")], pinocchio::SE3::Identity());
//         for (pinocchio::JointIndex jid = 1; jid < fixed_model.njoints; ++jid) {
//             model_.addJoint(model_.getJointId("free_flyer"), fixed_model.joints[jid],
//                            fixed_model.jointPlacements[jid], fixed_model.names[jid]);
//             model_.appendBodyToJoint(model_.njoints - 1, fixed_model.inertias[jid], pinocchio::SE3::Identity());
//         }
//         for (const auto& frame : fixed_model.frames) {
//             model_.addFrame(frame);
//         }
//         data_ = pinocchio::Data(model_);
//         pinocchio::urdf::buildGeom(model_, URDF_PATH, pinocchio::COLLISION, collision_model_, MESH_DIR);
//         pinocchio::urdf::buildGeom(model_, URDF_PATH, pinocchio::VISUAL, visual_model_, MESH_DIR);
//         collision_data_ = pinocchio::GeometryData(collision_model_);
//         visual_data_ = pinocchio::GeometryData(visual_model_);
//         if (model_.nq != 25 || model_.nv != 24) {
//             throw std::runtime_error("Unexpected DOF: nq=" + std::to_string(model_.nq) +
//                                      ", nv=" + std::to_string(model_.nv));
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Error in Pinocchio_WBC constructor: " << e.what() << std::endl;
//         throw;
//     }
// }

// void Pinocchio_WBC::pinocchio_test()
// {
//     // std::cout << "pinocchio_test" << std::endl;
//     // std::cout << "model name: " << model_.name << std::endl;

//     // 生成中立配置
//     Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
//     q.head(3) = Eigen::Vector3d(0.0, 0.0, 0.1); // 基座位置 (x, y, z)
//     q.segment(3, 4) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0); // 基座四元数 (x, y, z, w)
//     if (q.size() != 25) {
//         throw std::runtime_error("Generated q size (" + std::to_string(q.size()) +
//                                  ") does not match expected 25");
//     }
//     // std::cout << "q: " << q.transpose() << std::endl;

//     // 正向运动学
//     try {
//         pinocchio::forwardKinematics(model_, data_, q);
//         pinocchio::updateGeometryPlacements(model_, data_, collision_model_, collision_data_);
//         pinocchio::updateGeometryPlacements(model_, data_, visual_model_, visual_data_);
//     } catch (const std::exception& e) {
//         throw std::runtime_error("Error in forwardKinematics: " + std::string(e.what()));
//     }

//     // // 输出关节位置
//     // std::cout << "\nJoint placements:" << std::endl;
//     // for (pinocchio::JointIndex joint_id = 0; joint_id < static_cast<pinocchio::JointIndex>(model_.njoints); ++joint_id) {
//     //     std::cout << std::setw(24) << std::left << model_.names[joint_id] << ": " << std::fixed
//     //               << std::setprecision(2) << data_.oMi[joint_id].translation().transpose() << std::endl;
//     // }

//     // // 输出碰撞几何位置
//     // std::cout << "\nCollision object placements:" << std::endl;
//     // for (pinocchio::GeomIndex geom_id = 0; geom_id < collision_model_.ngeoms; ++geom_id) {
//     //     std::cout << geom_id << ": " << std::fixed << std::setprecision(2)
//     //               << collision_data_.oMg[geom_id].translation().transpose() << std::endl;
//     // }

//     // // 输出可视化几何位置
//     // std::cout << "\nVisual object placements:" << std::endl;
//     // for (pinocchio::GeomIndex geom_id = 0; geom_id < visual_model_.ngeoms; ++geom_id) {
//     //     std::cout << geom_id << ": " << std::fixed << std::setprecision(2)
//     //               << visual_data_.oMg[geom_id].translation().transpose() << std::endl;
//     // }
// }

// void Pinocchio_WBC::compute_wbc_matrices(
//     const Eigen::VectorXd& q,
//     const Eigen::VectorXd& v,
//     const std::vector<int>& support_legs,
//     const std::vector<int>& swing_legs,
//     Eigen::MatrixXd& J_1,
//     Eigen::MatrixXd& Jd_2,
//     Eigen::MatrixXd& Jd_3,
//     Eigen::MatrixXd& J_4,
//     Eigen::MatrixXd& Jd_4,
//     Eigen::MatrixXd& M,
//     Eigen::VectorXd& C,
//     Eigen::MatrixXd& Jc
// ) {
//     // 验证输入
//     if (q.size() != 25 || v.size() != 24) {
//         throw std::runtime_error("Invalid q size (" + std::to_string(q.size()) +
//                                  ") or v size (" + std::to_string(v.size()) + ")");
//     }
//     if (support_legs.size() != 3 || swing_legs.size() != 3) {
//         throw std::runtime_error("Expected 3 support legs and 3 swing legs");
//     }

//     // 初始化矩阵
//     J_1.resize(9, 24); J_1.setZero();
//     Jd_2.resize(3, 24); Jd_2.setZero(); // 旋转任务：3 行
//     Jd_3.resize(3, 24); Jd_3.setZero(); // 平移任务：3 行
//     J_4.resize(9, 24); J_4.setZero();
//     Jd_4.resize(9, 24); Jd_4.setZero();
//     M.resize(24, 24); M.setZero();
//     C.resize(24); C.setZero();
//     Jc.resize(9, 24); Jc.setZero();

//     // Pinocchio 计算
//     pinocchio::forwardKinematics(model_, data_, q, v);
//     pinocchio::computeJointJacobians(model_, data_, q);
//     pinocchio::updateFramePlacements(model_, data_);

//     std::vector<std::string> leg_frame_names = {
//         "leg_0_end", "leg_1_end", "leg_2_end",
//         "leg_3_end", "leg_4_end", "leg_5_end"
//     };

//     // 计算 J_1 和 Jc（支撑腿雅可比）
//     for (size_t i = 0; i < support_legs.size(); ++i) {
//         int leg_id = support_legs[i];
//         if (leg_id < 0 || leg_id >= 6) {
//             throw std::runtime_error("Invalid support leg ID: " + std::to_string(leg_id));
//         }
//         pinocchio::FrameIndex frame_id = model_.getFrameId(leg_frame_names[leg_id]);
//         if (frame_id >= model_.frames.size()) {
//             throw std::runtime_error("Frame not found: " + leg_frame_names[leg_id]);
//         }
//         Eigen::MatrixXd J_leg(6, model_.nv);
//         pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_leg);
//         J_1.block(3 * i, 0, 3, 24) = J_leg.topRows(3);
//         Jc.block(3 * i, 0, 3, 24) = J_leg.topRows(3);
//     }

//     // 检查 J_1 和 Jc 溢出
//     for (int i = 0; i < J_1.rows(); ++i) {
//         for (int j = 0; j < J_1.cols(); ++j) {
//             if (std::fabs(J_1(i, j)) > 10000) {
//                 J_1(i, j) = 0.0;
//                 // std::cerr << "Warning: J_1(" << i << "," << j << ") overflow reset to 0" << std::endl;
//             }
//             if (std::fabs(Jc(i, j)) > 10000) {
//                 Jc(i, j) = 0.0;
//                 // std::cerr << "Warning: Jc(" << i << "," << j << ") overflow reset to 0" << std::endl;
//             }
//         }
//     }

//     // 计算 J_4（摆动腿雅可比）
//     for (size_t i = 0; i < swing_legs.size(); ++i) {
//         int leg_id = swing_legs[i];
//         if (leg_id < 0 || leg_id >= 6) {
//             throw std::runtime_error("Invalid swing leg ID: " + std::to_string(leg_id));
//         }
//         pinocchio::FrameIndex frame_id = model_.getFrameId(leg_frame_names[leg_id]);
//         if (frame_id >= model_.frames.size()) {
//             throw std::runtime_error("Frame not found: " + leg_frame_names[leg_id]);
//         }
//         Eigen::MatrixXd J_leg(6, model_.nv);
//         pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_leg);
//         J_4.block(3 * i, 0, 3, 24) = J_leg.topRows(3);
//     }

//     // 检查 J_4 溢出
//     for (int i = 0; i < J_4.rows(); ++i) {
//         for (int j = 0; j < J_4.cols(); ++j) {
//             if (std::fabs(J_4(i, j)) > 10000) {
//                 J_4(i, j) = 0.0;
//                 // std::cerr << "Warning: J_4(" << i << "," << j << ") overflow reset to 0" << std::endl;
//             }
//         }
//     }

//     // 计算 Jd_4（摆动腿雅可比导数）
//     double eps = 1e-4;
//     for (size_t i = 0; i < swing_legs.size(); ++i) {
//         int leg_id = swing_legs[i];
//         pinocchio::FrameIndex frame_id = model_.getFrameId(leg_frame_names[leg_id]);
//         Eigen::MatrixXd J_leg(6, model_.nv);
//         pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_leg);
//         Eigen::MatrixXd J_leg_base = J_leg.topRows(3);

//         Eigen::VectorXd q_eps = q;
//         Eigen::MatrixXd J_leg_eps(6, model_.nv);
//         for (int j = 0; j < model_.nv; ++j) {
//             int q_idx = (j < 6) ? j : (j + 1); // 跳过四元数第 4 维
//             q_eps(q_idx) += eps;
//             if (j < 6) q_eps.segment(3, 4).normalize(); // 归一化四元数
//             pinocchio::forwardKinematics(model_, data_, q_eps);
//             pinocchio::computeJointJacobians(model_, data_, q_eps);
//             pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_leg_eps);
//             Eigen::MatrixXd diff = (J_leg_eps.topRows(3) - J_leg_base) / eps;
//             if (diff.norm() > 1e6 || !diff.allFinite()) {
//                 diff.setZero();
//                 // std::cerr << "Warning: Jd_4 diff overflow at leg " << leg_id << ", col " << j << std::endl;
//             }
//             Jd_4.block(3 * i, j, 3, 1) = diff.col(j);
//             q_eps(q_idx) = q(q_idx);
//         }
//     }

//     // 检查 Jd_4 溢出（修复检查对象）
//     for (int i = 0; i < Jd_4.rows(); ++i) {
//         for (int j = 0; j < Jd_4.cols(); ++j) {
//             if (std::fabs(Jd_4(i, j)) > 10000) { // 检查 Jd_4 而非 J_4
//                 Jd_4(i, j) = 0.0;
//                 // std::cerr << "Warning: Jd_4(" << i << "," << j << ") overflow reset to 0" << std::endl;
//             }
//         }
//     }

//     // 计算 Jd_2（机身旋转任务，3x24）
//     {
//         pinocchio::FrameIndex frame_id = model_.getFrameId("base_link");
//         if (frame_id >= model_.frames.size()) {
//             throw std::runtime_error("Frame not found: base_link");
//         }
//         Eigen::MatrixXd J_body(6, model_.nv);
//         pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_body);
//         Eigen::MatrixXd J_body_base = J_body.bottomRows(3); // 取旋转部分

//         Eigen::VectorXd q_eps = q;
//         Eigen::MatrixXd J_body_eps(6, model_.nv);
//         for (int j = 0; j < model_.nv; ++j) {
//             int q_idx = (j < 6) ? j : (j + 1);
//             q_eps(q_idx) += eps;
//             if (j < 6) q_eps.segment(3, 4).normalize();
//             pinocchio::forwardKinematics(model_, data_, q_eps);
//             pinocchio::computeJointJacobians(model_, data_, q_eps);
//             pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_body_eps);
//             Eigen::MatrixXd diff = (J_body_eps.bottomRows(3) - J_body_base) / eps;
//             if (diff.norm() > 1e6 || !diff.allFinite()) {
//                 diff.setZero();
//                 // std::cerr << "Warning: Jd_2 diff overflow at col " << j << std::endl;
//             }
//             Jd_2.block(0, j, 3, 1) = diff.col(j);
//             q_eps(q_idx) = q(q_idx);
//         }
//     }

//     // 检查 Jd_2 溢出
//     for (int i = 0; i < Jd_2.rows(); ++i) {
//         for (int j = 0; j < Jd_2.cols(); ++j) {
//             if (std::fabs(Jd_2(i, j)) > 10000) {
//                 Jd_2(i, j) = 0.0;
//                 // std::cerr << "Warning: Jd_2(" << i << "," << j << ") overflow reset to 0" << std::endl;
//             }
//         }
//     }

//     // 计算 Jd_3（机身平移任务，3x24）
//     {
//         pinocchio::centerOfMass(model_, data_, q);
//         Eigen::MatrixXd J_com(3, model_.nv);
//         J_com.setZero();
//         double total_mass = 0.0;
//         for (pinocchio::JointIndex jid = 1; jid < model_.njoints; ++jid) {
//             double mass = model_.inertias[jid].mass();
//             total_mass += mass;
//             Eigen::MatrixXd J_j(6, model_.nv);
//             pinocchio::getJointJacobian(model_, data_, jid, pinocchio::LOCAL_WORLD_ALIGNED, J_j);
//             J_com += mass * J_j.topRows(3);
//         }
//         if (total_mass < 1e-6) {
//             throw std::runtime_error("Invalid total mass: " + std::to_string(total_mass));
//         }
//         J_com /= total_mass;
//         Eigen::MatrixXd J_com_base = J_com;

//         Eigen::VectorXd q_eps = q;
//         Eigen::MatrixXd J_com_eps(3, model_.nv);
//         for (int j = 0; j < model_.nv; ++j) {
//             int q_idx = (j < 6) ? j : (j + 1);
//             q_eps(q_idx) += eps;
//             if (j < 6) q_eps.segment(3, 4).normalize();
//             pinocchio::forwardKinematics(model_, data_, q_eps);
//             pinocchio::computeJointJacobians(model_, data_, q_eps);
//             J_com_eps.setZero();
//             total_mass = 0.0;
//             for (pinocchio::JointIndex jid = 1; jid < model_.njoints; ++jid) {
//                 double mass = model_.inertias[jid].mass();
//                 total_mass += mass;
//                 Eigen::MatrixXd J_j(6, model_.nv);
//                 pinocchio::getJointJacobian(model_, data_, jid, pinocchio::LOCAL_WORLD_ALIGNED, J_j);
//                 J_com_eps += mass * J_j.topRows(3);
//             }
//             if (total_mass < 1e-6) {
//                 throw std::runtime_error("Invalid total mass in Jd_3: " + std::to_string(total_mass));
//             }
//             J_com_eps /= total_mass;
//             Eigen::MatrixXd diff = (J_com_eps - J_com_base) / eps;
//             if (diff.norm() > 1e6 || !diff.allFinite()) {
//                 diff.setZero();
//                 // std::cerr << "Warning: Jd_3 diff overflow at col " << j << std::endl;
//             }
//             Jd_3.block(0, j, 3, 1) = diff.col(j);
//             q_eps(q_idx) = q(q_idx);
//         }
//     }

//     // 检查 Jd_3 溢出
//     for (int i = 0; i < Jd_3.rows(); ++i) {
//         for (int j = 0; j < Jd_3.cols(); ++j) {
//             if (std::fabs(Jd_3(i, j)) > 10000) {
//                 Jd_3(i, j) = 0.0;
//                 // std::cerr << "Warning: Jd_3(" << i << "," << j << ") overflow reset to 0" << std::endl;
//             }
//         }
//     }

//     // 计算 M 和 C
//     pinocchio::crba(model_, data_, q);
//     M = data_.M;
//     pinocchio::nonLinearEffects(model_, data_, q, v);
//     C = data_.nle;
// }

// void Pinocchio_WBC::test_wbc_matrices()
// {
//     // std::cout << "Testing compute_wbc_matrices..." << std::endl;

//     // try {
//     //     Eigen::VectorXd q = Eigen::VectorXd::Zero(25);
//     //     q.head(3) = Eigen::Vector3d(0.0, 0.0, 0.1);
//     //     q.segment(3, 4) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
//     //     Eigen::VectorXd v = Eigen::VectorXd::Zero(24);
//     //     std::vector<int> support_legs = {0, 1, 2};
//     //     std::vector<int> swing_legs = {3, 4, 5};
//     //     Eigen::MatrixXd J_1, Jd_2, Jd_3, J_4, Jd_4, M, Jc;
//     //     Eigen::VectorXd C;

//     //     compute_wbc_matrices(q, v, support_legs, swing_legs, J_1, Jd_2, Jd_3, J_4, Jd_4, M, C, Jc);

//     //     std::cout << "J_1 size: " << J_1.rows() << "x" << J_1.cols() << std::endl;
//     //     std::cout << "Jd_2 size: " << Jd_2.rows() << "x" << Jd_2.cols() << std::endl;
//     //     std::cout << "Jd_3 size: " << Jd_3.rows() << "x" << Jd_3.cols() << std::endl;
//     //     std::cout << "J_4 size: " << J_4.rows() << "x" << J_4.cols() << std::endl;
//     //     std::cout << "Jd_4 size: " << Jd_4.rows() << "x" << Jd_4.cols() << std::endl;
//     //     std::cout << "M size: " << M.rows() << "x" << M.cols() << std::endl;
//     //     std::cout << "C size: " << C.size() << std::endl;
//     //     std::cout << "Jc size: " << Jc.rows() << "x" << Jc.cols() << std::endl;

//     //     // 输出矩阵样本
//     //     std::cout << "J_1 :\n" << std::setw(5) << J_1 << "\n" << std::endl;
//     //     std::cout << "Jd_2 :\n" << std::setw(5) << Jd_2 << "\n" << std::endl;
//     //     std::cout << "Jd_3 :\n" << std::setw(5) << Jd_3 << "\n" << std::endl;
//     //     std::cout << "J_4:\n" << std::setw(5) << J_4 << "\n" << std::endl;
//     //     std::cout << "Jd_4 :\n" << std::setw(5) << Jd_4 << "\n" << std::endl;
//     //     std::cout << "M :\n" << std::setw(5) << M << "\n" << std::endl;
//     //     std::cout << "C :\n" << std::setw(5) << C<< "\n" << std::endl;
//     //     std::cout << "Jc :\n" << std::setw(5) << Jc << "\n" << std::endl;

//     //     std::cout << "Test completed successfully." << std::endl;
//     // } catch (const std::exception& e) {
//     //     std::cerr << "Test failed: " << e.what() << std::endl;
//     // }
// }