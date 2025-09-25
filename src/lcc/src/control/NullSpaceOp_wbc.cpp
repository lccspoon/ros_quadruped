#include "control/NullSpaceOp_wbc.h"


// 构造函数：初始化变量（示例值）
NullSpaceOp_wbc::NullSpaceOp_wbc(/* args */) {
    M.setIdentity();  // 质量矩阵初始化为单位矩阵（示例）
    h.setZero();      // 非线性项初始化为零

    J_support.setRandom();
    J_body_rot.setRandom();
    J_body_trans.setRandom();
    J_swing.setRandom();

    q.setZero();
    v.setZero();

    mpc_qdd.setRandom();
    mpc_f.setRandom();

    delta_qdd.setZero();
    delta_f.setZero();

    v_support_des.setZero();         // 支撑腿静止
    v_body_rot_des << 0.1, 0, 0;     // 轻微旋转
    v_body_trans_des << 0, 0, 0.01;  // 轻微平动
    v_swing_des.setRandom();         // 摆动腿随机目标
}

// 手动计算伪逆（基于 SVD）
template <int Rows, int Cols>
Eigen::Matrix<double, Cols, Rows> NullSpaceOp_wbc::compute_pseudo_inverse(
    const Eigen::Matrix<double, Rows, Cols>& A
) {
    Eigen::JacobiSVD<Eigen::Matrix<double, Rows, Cols>> svd(
        A, Eigen::ComputeFullU | Eigen::ComputeFullV
    );
    Eigen::Matrix<double, Rows, Rows> U = svd.matrixU();
    Eigen::Matrix<double, Cols, Cols> V = svd.matrixV();
    Eigen::Matrix<double, Rows, Cols> Sigma = Eigen::Matrix<double, Rows, Cols>::Zero();
    
    // 获取奇异值
    Eigen::VectorXd singularValues = svd.singularValues();
    double epsilon = 1e-10; // 数值稳定性阈值
    for (int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > epsilon) {
            Sigma(i, i) = 1.0 / singularValues(i); // 倒数
        }
    }
    
    // 伪逆 = V * Sigma^+ * U^T
    return V * Sigma.transpose() * U.transpose();
}

// 任务 1：支撑腿跟随
void NullSpaceOp_wbc::compute_support_task(
    Eigen::Matrix<double, 24, 1>& qdd,
    Eigen::Matrix<double, 24, 24>& N_out,
    const Eigen::Matrix<double, 24, 24>& N_in
) {
    Eigen::Matrix<double, 9, 24> J_N = J_support * N_in;
    Eigen::Matrix<double, 24, 9> J_pinv = compute_pseudo_inverse(J_N); // 手动伪逆
    qdd = J_pinv * (v_support_des - J_support * v);
    N_out = N_in * (Eigen::Matrix<double, 24, 24>::Identity() - J_pinv * J_support);
}

// 任务 2：机身旋转
void NullSpaceOp_wbc::compute_body_rot_task(
    Eigen::Matrix<double, 24, 1>& qdd,
    Eigen::Matrix<double, 24, 24>& N_out,
    const Eigen::Matrix<double, 24, 24>& N_in
) {
    Eigen::Matrix<double, 3, 24> J_N = J_body_rot * N_in;
    Eigen::Matrix<double, 24, 3> J_pinv = compute_pseudo_inverse(J_N); // 手动伪逆
    qdd = J_pinv * (v_body_rot_des - J_body_rot * v);
    N_out = N_in * (Eigen::Matrix<double, 24, 24>::Identity() - J_pinv * J_body_rot);
}

// 任务 3：机身平动
void NullSpaceOp_wbc::compute_body_trans_task(
    Eigen::Matrix<double, 24, 1>& qdd,
    Eigen::Matrix<double, 24, 24>& N_out,
    const Eigen::Matrix<double, 24, 24>& N_in
) {
    Eigen::Matrix<double, 3, 24> J_N = J_body_trans * N_in;
    Eigen::Matrix<double, 24, 3> J_pinv = compute_pseudo_inverse(J_N); // 手动伪逆
    qdd = J_pinv * (v_body_trans_des - J_body_trans * v);
    N_out = N_in * (Eigen::Matrix<double, 24, 24>::Identity() - J_pinv * J_body_trans);
}

// 任务 4：摆动腿跟随
void NullSpaceOp_wbc::compute_swing_task(
    Eigen::Matrix<double, 24, 1>& qdd,
    Eigen::Matrix<double, 24, 24>& N_out,
    const Eigen::Matrix<double, 24, 24>& N_in
) {
    Eigen::Matrix<double, 9, 24> J_N = J_swing * N_in;
    Eigen::Matrix<double, 24, 9> J_pinv = compute_pseudo_inverse(J_N); // 手动伪逆
    qdd = J_pinv * (v_swing_des - J_swing * v);
    N_out = N_in * (Eigen::Matrix<double, 24, 24>::Identity() - J_pinv * J_swing);
}

// grok 写的
// 零空间优化主函数
void NullSpaceOp_wbc::compute_nullspace_optimization(
    Eigen::Matrix<double, 24, 1>& q_out,
    Eigen::Matrix<double, 24, 1>& v_out,
    Eigen::Matrix<double, 24, 1>& qdd_out
) {
    Eigen::Matrix<double, 24, 24> N = Eigen::Matrix<double, 24, 24>::Identity();
    Eigen::Matrix<double, 24, 1> qdd = Eigen::Matrix<double, 24, 1>::Zero();

    Eigen::Matrix<double, 24, 1> qdd_task1;
    Eigen::Matrix<double, 24, 24> N_task1;
    compute_support_task(qdd_task1, N_task1, N);
    qdd += qdd_task1;
    N = N_task1;

    Eigen::Matrix<double, 24, 1> qdd_task2;
    Eigen::Matrix<double, 24, 24> N_task2;
    compute_body_rot_task(qdd_task2, N_task2, N);
    qdd += qdd_task2;
    N = N_task2;

    Eigen::Matrix<double, 24, 1> qdd_task3;
    Eigen::Matrix<double, 24, 24> N_task3;
    compute_body_trans_task(qdd_task3, N_task3, N);
    qdd += qdd_task3;
    N = N_task3;

    Eigen::Matrix<double, 24, 1> qdd_task4;
    Eigen::Matrix<double, 24, 24> N_task4;
    compute_swing_task(qdd_task4, N_task4, N);
    qdd += qdd_task4;

    qdd_out = qdd;
    v_out = v + qdd * 0.01; // 假设时间步长 dt = 0.01
    q_out = q + v_out * 0.01; // 数值积分得到关节角度
}

// created by lcc 20250411
void NullSpaceOp_wbc::compute_nullspace_optimization_lcc( 
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

){
    Eigen::Matrix<double, 24, 24> II;
    II.setIdentity();

    /*  i=1   */
    Eigen::Matrix<double, 24, 1> delta_q_1_cmd, qd_1_cmd, qdd_1_cmd; 
    delta_q_1_cmd.setZero();
    qd_1_cmd.setZero();
    qdd_1_cmd = compute_pseudo_inverse(J_1) * ( (-1) * J_1 * qd );

    /*  for i = 2 to 4 do:  */

    /*  i=2   */
    Eigen::Matrix<double, 3, 1> e2 = x_2_d - x_2;//eq(1)

    Eigen::Matrix<double, 3, 3> kp_2, kd_2;
    kp_2.setZero();
    kd_2.setZero();
    kp_2 = Eigen::Matrix<double, 3, 1>(1, 1, 1).asDiagonal();
    kd_2 = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1).asDiagonal();
    Eigen::Matrix<double, 3, 1> xdd_2_cmd = xdd_2_d + kp_2*(x_2_d-x_2) + kd_2*(xd_2_d-xd_2);//eq(2)

    Eigen::Matrix<double, 9, 24> J_1_A = J_1;//eq(3) 带A的都是i-1

    Eigen::Matrix<double, 24, 24> N_1_A = II - compute_pseudo_inverse(J_1_A) * J_1_A;//eq(4)

    Eigen::Matrix<double, 3, 24> J_N_2 = J_2 * N_1_A;
    Eigen::Matrix<double, 24, 3> J_N_2_ps = compute_pseudo_inverse(J_N_2);//eq(4.1)

    Eigen::Matrix<double, 24, 1> delta_q_2_cmd = delta_q_1_cmd + J_N_2_ps * (e2 - J_2 * delta_q_1_cmd);//eq(5)

    Eigen::Matrix<double, 24, 1> qd_2_cmd = qd_1_cmd + J_N_2_ps * (x_2_d - J_2 * qd_1_cmd);//eq(6)

    Eigen::Matrix<double, 24, 1> qdd_2_cmd = qdd_1_cmd + J_N_2_ps * (xdd_2_cmd - Jd_2 * qd - J_2 * qdd_1_cmd);//eq(7)

    /*  i=3   */
    Eigen::Matrix<double, 3, 1> e3 = x_3_d - x_3;//eq(1)

    Eigen::Matrix<double, 3, 3> kp_3, kd_3;
    kp_3.setZero();
    kd_3.setZero();
    kp_3 = Eigen::Matrix<double, 3, 1>(1, 1, 1).asDiagonal();
    kd_3 = Eigen::Matrix<double, 3, 1>(0.1, 0.1, 0.1).asDiagonal();
    Eigen::Matrix<double, 3, 1> xdd_3_cmd = xdd_3_d + kp_3*(x_3_d-x_3) + kd_3*(xd_3_d-xd_3);//eq(2)

    Eigen::Matrix<double, 12, 24> J_2_A;//eq(3)
    J_2_A.block<9, 24>(0, 0) = J_1;
    J_2_A.block<3, 24>(9, 0) = J_2;

    Eigen::Matrix<double, 24, 24> N_2_A = II - compute_pseudo_inverse(J_2_A) * J_2_A;//eq(4) 带A的都是i-1

    Eigen::Matrix<double, 3, 24> J_N_3 = J_3 * N_2_A;
    Eigen::Matrix<double, 24, 3> J_N_3_ps = compute_pseudo_inverse(J_N_3);//eq(4.1)

    Eigen::Matrix<double, 24, 1> delta_q_3_cmd = delta_q_2_cmd + J_N_3_ps * (e3 - J_3 * delta_q_2_cmd);//eq(5)

    Eigen::Matrix<double, 24, 1> qd_3_cmd = qd_2_cmd + J_N_3_ps * (x_3_d - J_3 * qd_2_cmd);//eq(6)

    Eigen::Matrix<double, 24, 1> qdd_3_cmd = qdd_2_cmd + J_N_3_ps * (xdd_3_cmd - Jd_3 * qd - J_3 * qdd_2_cmd);//eq(7)

    /*  i=4   */
    Eigen::Matrix<double, 9, 1> e4 = x_4_d - x_4;//eq(1)

    Eigen::Matrix<double, 9, 9> kp_4, kd_4;
    kp_4.setIdentity();
    kd_4.setIdentity();
    // kp_4 = Eigen::Matrix<double, 9, 1>(1, 1, 1, 1, 1, 1, 1, 1, 1).asDiagonal();
    // kd_4 = Eigen::Matrix<double, 9, 1>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).asDiagonal();
    Eigen::Matrix<double, 9, 1> xdd_4_cmd = xdd_4_d + kp_4*(x_4_d-x_4) + kd_4*(xd_4_d-xd_4) * 0.1;//eq(2)

    Eigen::Matrix<double, 15, 24> J_3_A;//eq(3)
    J_3_A.block<9, 24>(0, 0) = J_1;
    J_3_A.block<3, 24>(9, 0) = J_2;
    J_3_A.block<3, 24>(12, 0) = J_3;

    Eigen::Matrix<double, 24, 24> N_3_A = II - compute_pseudo_inverse(J_3_A) * J_3_A;//eq(4) 带A的都是i-1

    Eigen::Matrix<double, 9, 24> J_N_4 = J_4 * N_3_A;
    Eigen::Matrix<double, 24, 9> J_N_4_ps = compute_pseudo_inverse(J_N_4);//eq(4.1)

    Eigen::Matrix<double, 24, 1> delta_q_4_cmd = delta_q_3_cmd + J_N_4_ps * (e4 - J_4 * delta_q_3_cmd);//eq(5)

    Eigen::Matrix<double, 24, 1> qd_4_cmd = qd_3_cmd + J_N_4_ps * (x_4_d - J_4 * qd_3_cmd);//eq(6)

    Eigen::Matrix<double, 24, 1> qdd_4_cmd = qdd_3_cmd + J_N_4_ps * (xdd_4_cmd - Jd_4 * qd - J_4 * qdd_3_cmd);//eq(7)

    /*  out   */
    q_cmd = q + delta_q_4_cmd;
    qd_cmd = q + qd_4_cmd;
    qdd_cmd = q + qdd_4_cmd;

    // std::cout << "q_cmd: " << q_cmd.transpose() << std::endl;
    // std::cout << "qd_cmd: " << qd_cmd.transpose() << std::endl;
    // std::cout << "qdd_cmd: " << qd_cmd.transpose() << std::endl;


}



void NullSpaceOp_wbc::compute_nullspace_optimization_lcc_test(void)
{
    // 初始化输入参数
    Eigen::Matrix<double, 24, 1> q = Eigen::Matrix<double, 24, 1>::Zero(); // 关节位置
    Eigen::Matrix<double, 24, 1> qd = Eigen::Matrix<double, 24, 1>::Ones() * 0.1; // 关节速度
    Eigen::Matrix<double, 9, 24> J_1 = Eigen::Matrix<double, 9, 24>::Random(); // 任务1雅可比
    Eigen::Matrix<double, 3, 1> x_2 = Eigen::Matrix<double, 3, 1>::Zero(); // 任务2当前位置
    Eigen::Matrix<double, 3, 1> x_2_d = Eigen::Matrix<double, 3, 1>::Ones() * 0.5; // 任务2期望位置
    Eigen::Matrix<double, 3, 1> xd_2 = Eigen::Matrix<double, 3, 1>::Zero(); // 任务2当前速度
    Eigen::Matrix<double, 3, 1> xd_2_d = Eigen::Matrix<double, 3, 1>::Zero(); // 任务2期望速度
    Eigen::Matrix<double, 3, 1> xdd_2_d = Eigen::Matrix<double, 3, 1>::Zero(); // 任务2期望加速度
    Eigen::Matrix<double, 3, 24> J_2 = Eigen::Matrix<double, 3, 24>::Random(); // 任务2雅可比
    Eigen::Matrix<double, 3, 24> Jd_2 = Eigen::Matrix<double, 3, 24>::Zero(); // 任务2雅可比导数
    Eigen::Matrix<double, 3, 1> x_3 = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 1> x_3_d = Eigen::Matrix<double, 3, 1>::Ones() * 0.3;
    Eigen::Matrix<double, 3, 1> xd_3 = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 1> xd_3_d = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 1> xdd_3_d = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 24> J_3 = Eigen::Matrix<double, 3, 24>::Random();
    Eigen::Matrix<double, 3, 24> Jd_3 = Eigen::Matrix<double, 3, 24>::Zero();
    Eigen::Matrix<double, 9, 1> x_4 = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> x_4_d = Eigen::Matrix<double, 9, 1>::Ones() * 0.2;
    Eigen::Matrix<double, 9, 1> xd_4 = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> xd_4_d = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> xdd_4_d = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 24> J_4 = Eigen::Matrix<double, 9, 24>::Random();
    Eigen::Matrix<double, 9, 24> Jd_4 = Eigen::Matrix<double, 9, 24>::Zero();

    // 调用函数
    compute_nullspace_optimization_lcc(
        q, qd, J_1,
        x_2, x_2_d, xd_2, xd_2_d, xdd_2_d, J_2, Jd_2,
        x_3, x_3_d, xd_3, xd_3_d, xdd_3_d, J_3, Jd_3,
        x_4, x_4_d, xd_4, xd_4_d, xdd_4_d, J_4, Jd_4
    );
}


