#ifndef _TERAINESTI_H_
#define _TERAINESTI_H_

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "control/ConvexMpc.h"
#include "control/TerrianEsti.h"
#include "common/filter.h"


class TerrianEsti
{
public:
    TerrianEsti(/* args */);
    ~TerrianEsti();

    Vec3 compute_walking_surface(VecInt4 *_contact, Vec34 _posFeet2BGlobal, Vec3 *_Apla) ;
    Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    void terrain_adaptation(Vec3 _posBody, double _yawCmd, Vec3 &root_euler_d, VecInt4 *_contact, Vec34 _posFeet2BGlobal, Vec3 *_Apla );
    double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);

private:
    //terrian estimator
    Eigen::Matrix<double, 3, 4> foot_pos_recent_contact;
    MovingWindowFilter terrain_angle_filter;
    MovingWindowFilter root_euler_d_x_filter;
    MovingWindowFilter root_euler_d_y_filter;
    MovingWindowFilter recent_contact_x_filter[4];
    MovingWindowFilter recent_contact_y_filter[4];
    MovingWindowFilter recent_contact_z_filter[4];
    int use_terrain_adapt = 1 ;
    Vec3 *_Apla;

};


#endif  // BALANCECTRL_H