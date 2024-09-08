/*
    @author lcc
    @date 20240419
    @brief 斜坡地形估计。
    @cite 参考代码：yy硕。 参考论文：yxy。
*/

#include "control/TerrianEsti.h"

Vec2 TERRIAN_EST_DEGREE;

// #if TERRIANESTI_FOURLEG
TerrianEsti::TerrianEsti(/* args */)
{
    foot_pos_recent_contact.setZero();
    terrain_angle_filter = MovingWindowFilter(100);
    root_euler_d_x_filter = MovingWindowFilter(100);
    root_euler_d_y_filter = MovingWindowFilter(100);
    for (int i = 0; i < 4; ++i) {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
}
TerrianEsti::~TerrianEsti(){}
void TerrianEsti::terrain_adaptation(Vec3 _posBody, double _yawCmd, Vec3 &root_euler_d, VecInt4 *_contact, Vec34 _posFeet2BGlobal, Vec3 *_Apla ){
        Eigen::Vector3d surf_coef = compute_walking_surface(_contact, _posFeet2BGlobal, _Apla);
        Eigen::Vector3d flat_ground_coef;
        flat_ground_coef << 0, 0, 1;
        double terrain_angle = 0;
        // only record terrain angle when the body is high
        if (_posBody(2) > 0.1) {
            terrain_angle = terrain_angle_filter.CalculateAverage(cal_dihedral_angle(flat_ground_coef, surf_coef));
        } else {
            terrain_angle = 0;
        }

        if (terrain_angle > 0.5) {
            terrain_angle = 0.5;
        }
        if (terrain_angle < -0.5) {
            terrain_angle = -0.5;
        }

        /* ------ yxy ---  */
        surf_coef = -surf_coef;
        Vec3 ne = surf_coef.normalized(); // 求n的单位向量
        Vec3 kethai;
        Mat3 gama;
        gama <<
        cos(_yawCmd), -sin(_yawCmd), 0,
        sin(_yawCmd), cos(_yawCmd), 0,
        0, 0, 1;
        kethai = pseudo_inverse(gama.transpose() * gama) * gama.transpose() * ne;  // 对应 yxy (2.70)
        root_euler_d(0) = -root_euler_d_x_filter.CalculateAverage( asin( kethai(1) )); //lcc 20240617：相比于yxy论文，添加了负号"-"
        root_euler_d(1) = root_euler_d_y_filter.CalculateAverage( atan( kethai(0) / kethai(2) ) );
        // root_euler_d = root_euler_d * 1.0;//lcc 20240904
        TERRIAN_EST_DEGREE << root_euler_d(0), root_euler_d(1);
}

Vec3 TerrianEsti::compute_walking_surface(VecInt4 *_contact, Vec34 _posFeet2BGlobal, Vec3 *_Apla) {

    for (int i = 0; i < 4; ++i)
    if ((*_contact)(i) == 1) {
            foot_pos_recent_contact.block<3, 1>(0, i)
                    << recent_contact_x_filter[i].CalculateAverage(_posFeet2BGlobal(0, i)),
                    recent_contact_y_filter[i].CalculateAverage(_posFeet2BGlobal(1, i)),
                    recent_contact_z_filter[i].CalculateAverage(_posFeet2BGlobal(2, i));
    }

    Eigen::Matrix<double, 4, 3> W;  // yxy (2.69) 中的 Wpla
    Eigen::VectorXd foot_pos_z; // yxy (2.68) 
    Eigen::Vector3d surf_coef;

    W.block<4, 1>(0, 0).setOnes();
    W.block<4, 2>(0, 1) = foot_pos_recent_contact.block<2, 4>(0, 0).transpose();

    foot_pos_z.resize(4);
    foot_pos_z = foot_pos_recent_contact.block<1, 4>(2, 0).transpose();
    (*_Apla) = pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;  // 对应 yxy (2.70)
    surf_coef << (*_Apla)(1), (*_Apla)(2), -1;
    return surf_coef;
}
// #else
// TerrianEsti::TerrianEsti(/* args */)
// {
//     foot_pos_recent_contact.setZero();
//     terrain_angle_filter = MovingWindowFilter(100);
//     root_euler_d_x_filter = MovingWindowFilter(100);
//     root_euler_d_y_filter = MovingWindowFilter(100);
//     for (int i = 0; i < 6; ++i) {
//         recent_contact_x_filter[i] = MovingWindowFilter(60);
//         recent_contact_y_filter[i] = MovingWindowFilter(60);
//         recent_contact_z_filter[i] = MovingWindowFilter(60);
//     }
// }
// TerrianEsti::~TerrianEsti(){}
// void TerrianEsti::terrain_adaptation(Vec3 _posBody, double _yawCmd, Vec3 &root_euler_d, VecInt6 *_contact, Vec36 _posFeet2BGlobal, Vec3 *_Apla ){
//         Eigen::Vector3d surf_coef = compute_walking_surface(_contact, _posFeet2BGlobal, _Apla);
//         Eigen::Vector3d flat_ground_coef;
//         flat_ground_coef << 0, 0, 1;
//         double terrain_angle = 0;
//         // only record terrain angle when the body is high
//         if (_posBody(2) > 0.1) {
//             terrain_angle = terrain_angle_filter.CalculateAverage(cal_dihedral_angle(flat_ground_coef, surf_coef));
//         } else {
//             terrain_angle = 0;
//         }

//         if (terrain_angle > 0.5) {
//             terrain_angle = 0.5;
//         }
//         if (terrain_angle < -0.5) {
//             terrain_angle = -0.5;
//         }
//         /* ------ yxy ---  */
//         surf_coef = -surf_coef;
//         Vec3 ne = surf_coef.normalized(); // 求n的单位向量
//         Vec3 kethai;
//         Mat3 gama;
//         gama <<
//         cos(_yawCmd), -sin(_yawCmd), 0,
//         sin(_yawCmd), cos(_yawCmd), 0,
//         0, 0, 1;
//         kethai = pseudo_inverse(gama.transpose() * gama) * gama.transpose() * ne;  // 对应 yxy (2.70)
//         root_euler_d(0) = root_euler_d_x_filter.CalculateAverage( asin( kethai(1) ));
//         root_euler_d(1) = root_euler_d_y_filter.CalculateAverage( atan( kethai(0) / kethai(2) ) );
// }

// Vec3 TerrianEsti::compute_walking_surface(VecInt6 *_contact, Vec36 _posFeet2BGlobal, Vec3 *_Apla) {

//     for (int i = 0; i < 6; ++i)
//     if ((*_contact)(i) == 1) {
//             foot_pos_recent_contact.block<3, 1>(0, i)
//                     << recent_contact_x_filter[i].CalculateAverage(_posFeet2BGlobal(0, i)),
//                     recent_contact_y_filter[i].CalculateAverage(_posFeet2BGlobal(1, i)),
//                     recent_contact_z_filter[i].CalculateAverage(_posFeet2BGlobal(2, i));
//     }

//     Eigen::Matrix<double, 6, 3> W;  // yxy (2.69) 中的 Wpla
//     Eigen::VectorXd foot_pos_z; // yxy (2.68) 
//     Eigen::Vector3d surf_coef;

//     W.block<6, 1>(0, 0).setOnes();
//     W.block<6, 2>(0, 1) = foot_pos_recent_contact.block<2, 6>(0, 0).transpose();

//     foot_pos_z.resize(6);
//     foot_pos_z = foot_pos_recent_contact.block<1, 6>(2, 0).transpose();

//     // Vec3 b;
//     // b = pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;  // 对应 yxy (2.70)
//     (*_Apla) = pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;  // 对应 yxy (2.70)
//     // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
//     surf_coef << (*_Apla)(1), (*_Apla)(2), -1;
//     // std::cout<<" (*_Apla): \n"<< (*_Apla).transpose() <<std::endl;
//     return surf_coef;
// }
// #endif


Eigen::Matrix3d TerrianEsti::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

double TerrianEsti::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}