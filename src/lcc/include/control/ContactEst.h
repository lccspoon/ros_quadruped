#ifndef CONTACT_DETECTION_SIMPLE
#define CONTACT_DETECTION_SIMPLE

#include<iostream>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <Eigen/Dense>
#include<math.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>


class ContactEst
{
    private:

        Eigen::Matrix<double,3,6> contact_estimate;
        Eigen::Matrix<double,1,6> contact_estimate_schedual;
        #define NUM_LEG 6 
        #define X_THRESHOLD 1 
        #define Y_THRESHOLD 1
        #define Z_THRESHOLD 1

        Eigen::Matrix<double,3,6> _footEndForce;
        Eigen::Matrix<double,3,6> _footEndForce_past;
        // ContactEst(){
        //     _footEndForce.setZero();
        //     _footEndForce_past.setZero();
        // }

        Eigen::Matrix<double,1,6> leg_swingphase_contact_est_count;
        Eigen::Matrix<double,1,6> leg_suportingphase_contact_est_count;

    public:
        ContactEst();
        ~ContactEst();
        Eigen::Matrix<double,1,6> leg_swingphase_contact_est;
        Eigen::Matrix<double,1,6> leg_suportingphase_contact_est;
        void simple_contact_est(Eigen::Matrix<double,3,6> footend_force,
                                                        Eigen::Matrix<int,1,6> contact,
                                                        Eigen::Matrix<double,1,6> phase,
                                                        Eigen::Matrix<double,1,6> swing_contact_threadhold,
                                                        double supor_contact_threadhold);
};



#endif
