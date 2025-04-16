#include "control/Control_WBC.h"
Control_WBC::Control_WBC(/* args */)
{
    // pinocchio_test();

    Eigen::Matrix<double, 24, 1> q_ns, v_ns, qdd_ns;
    q_ns.setZero();
    v_ns.setZero();
    qdd_ns.setZero();

    // ns_wbc.compute_nullspace_optimization(q_ns, v_ns, qdd_ns);
    // std::cout << "Joint angles (q_ns): " << q_ns.transpose() << std::endl;
    // std::cout << "Joint velocities (v_ns): " << v_ns.transpose() << std::endl;
    // std::cout << "Joint accelerations (qdd_ns): " << qdd_ns.transpose() << std::endl;

    // ns_wbc.compute_nullspace_optimization_lcc_test();

    // balance_wbc.runTest();
    // pino_wbc.pinocchio_test();
    // pino_wbc.test_wbc_matrices();



}

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


Eigen::Matrix<double, 18, 1> Control_WBC::wbc_run(
    Eigen::Matrix<double, 25, 1> _q_with_quat,
    Eigen::Matrix<double, 24, 1> _q_with_rpy,
    Eigen::Matrix<double, 24, 1> _qd,
    VecInt6 _contact_hex,
    Eigen::Matrix<double, 3, 1> x_2,
    Eigen::Matrix<double, 3, 1> x_2_d,
    Eigen::Matrix<double, 3, 1> xd_2,
    Eigen::Matrix<double, 3, 1> xd_2_d,
    Eigen::Matrix<double, 3, 24> J_2,
    Eigen::Matrix<double, 3, 1> x_3,
    Eigen::Matrix<double, 3, 1> x_3_d,
    Eigen::Matrix<double, 3, 1> xd_3,
    Eigen::Matrix<double, 3, 1> xd_3_d,
    Eigen::Matrix<double, 3, 24> J_3,
    Eigen::Matrix<double, 9, 1> x_4,
    Eigen::Matrix<double, 9, 1> x_4_d,
    Eigen::Matrix<double, 9, 1> xd_4,
    Eigen::Matrix<double, 9, 1> xd_4_d,
    Eigen::Matrix<double, 3, 6> forec_mpc
) {

    std::vector<int> support_legs = {0, 1, 2};
    std::vector<int> swing_legs = {3, 4, 5};

    Eigen::MatrixXd J_1, Jd_2, Jd_3, J_4, Jd_4, M, Jc;
    Eigen::VectorXd C;

    //仅限三角步态：
    VecInt6 contact;
    contact.setZero();
    if( _contact_hex(0) == 1  ) //支撑腿 0 3 4 
    {
        std::vector<int> swing_legs = {1, 2, 5};  
        std::vector<int> support_legs = {0, 3, 4};
        contact(0) = 1;
        contact(3) = 1;
        contact(4) = 1;

        contact(1) = 0;
        contact(2) = 0;
        contact(5) = 0;
    }
    else 
    {
        std::vector<int> support_legs = {1, 2, 5};  
        std::vector<int> swing_legs = {0, 3, 4};

        contact(0) = 0;
        contact(3) = 0;
        contact(4) = 0;

        contact(1) = 1;
        contact(2) = 1;
        contact(5) = 1;
    }

    pino_wbc.compute_wbc_matrices( _q_with_quat, _qd, support_legs, swing_legs, J_1, Jd_2, Jd_3, J_4, Jd_4, M, C, Jc);

    // 初始化任务参数
    Eigen::Matrix<double, 3, 1> xdd_2_d = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 1> xdd_3_d = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 9, 1> xdd_4_d = Eigen::Matrix<double, 9, 1>::Zero();

    ns_wbc.compute_nullspace_optimization_lcc(
        _q_with_rpy, _qd, J_1,
        x_2, x_2_d, xd_2, xd_2_d, xdd_2_d, J_2, Jd_2,
        x_3, x_3_d, xd_3, xd_3_d, xdd_3_d, J_3, Jd_3,
        x_4, x_4_d, xd_4, xd_4_d, xdd_4_d, J_4, Jd_4
    );

    // J_1 size: 9x24
    // Jd_2 size: 3x24
    // Jd_3 size: 3x24
    // J_4 size: 9x24
    // Jd_4 size: 9x24
    // M size: 24x24
    // C size: 24
    // Jc size: 9x24
    
    Vec24 qdd_cmd = Vec24::Zero();

    Eigen::Matrix<double, 9, 1> f_mpc_active;
    f_mpc_active.setZero();
    if (contact(0) == 1 ) { //支撑腿 0 3 4 
        f_mpc_active.segment<3>(0) = forec_mpc.block<3,1>(0,0);
        f_mpc_active.segment<3>(3) = forec_mpc.block<3,1>(0,3);
        f_mpc_active.segment<3>(6) = forec_mpc.block<3,1>(0,4);
    } 
    else 
    {   //支撑腿 1 2 5 
        f_mpc_active.segment<3>(0) = forec_mpc.block<3,1>(0,1);
        f_mpc_active.segment<3>(3) = forec_mpc.block<3,1>(0,2);
        f_mpc_active.segment<3>(6) = forec_mpc.block<3,1>(0,5);
    }

    // f_mpc_active(2) = 50;
    // f_mpc_active(5) = 50;
    // f_mpc_active(8) = 50;


    // balance_wbc.runTest();

    // std::cout << "f_mpc_active: " << f_mpc_active << std::endl;
    // std::cout << "M: " << M << std::endl;
    // std::cout << "C: " << C << std::endl;
    // std::cout << "Jc: " << Jc << std::endl;

    Eigen::Matrix<double, 9, 1> fc = balance_wbc.calF(qdd_cmd, f_mpc_active, contact, M, C, Jc.transpose());

//     std::cout << "f_mpc_active: " << f_mpc_active << std::endl;
//     std::cout << "aaTest fc: " << fc.transpose() << std::endl;


    Eigen::Matrix<double, 18, 1> f_return;
    f_return.setZero();
    if (contact(0) == 1 ) { //支撑腿 0 3 4 
        f_return.block<3,1>(3 * 0,0) = fc.block<3,1>(3 * 0,0);
        f_return.block<3,1>(3 * 3,0) = fc.block<3,1>(3 * 1,0);
        f_return.block<3,1>(3 * 4,0) = fc.block<3,1>(3 * 2,0);
    } 
    else 
    {   //支撑腿 1 2 5 
        f_return.block<3,1>(3 * 1,0) = fc.block<3,1>(3 * 0,0);
        f_return.block<3,1>(3 * 2,0) = fc.block<3,1>(3 * 1,0);
        f_return.block<3,1>(3 * 5,0) = fc.block<3,1>(3 * 2,0);
    }

    return f_return;
}

Eigen::Matrix<double, 18, 1> Control_WBC::ret_q_cmd(void)
{
    return  ns_wbc.q_cmd.block<18,1>(6,0);
}

Eigen::Matrix<double, 18, 1> Control_WBC::ret_qd_cmd(void)
{
    return  ns_wbc.qd_cmd.block<18,1>(6,0);
}