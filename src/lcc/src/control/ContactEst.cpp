#include "control/ContactEst.h"


ContactEst::ContactEst(){
    _footEndForce.setZero();
    _footEndForce_past.setZero();
    leg_swingphase_contact_est_count.setZero();
    leg_suportingphase_contact_est_count.setZero();
}

ContactEst::~ContactEst(){
    ;
}

void ContactEst::simple_contact_est(Eigen::Matrix<double,3,6> footend_force,
                                                        Eigen::Matrix<int,1,6> contact,
                                                        Eigen::Matrix<double,1,6> phase,
                                                        Eigen::Matrix<double,1,6> swing_contact_threadhold,
                                                        double supor_contact_threadhold)
{
    _footEndForce = footend_force;

    _footEndForce = 0.8 * _footEndForce_past + (1 - 0.8) * _footEndForce;
    _footEndForce_past = _footEndForce;
    // std::cout<<" _footEndForce \n:"<< _footEndForce <<std::endl;

    for (int i=0; i<6; ++i) {
        Eigen::Vector3d foot_for = _footEndForce.block<3,1>(0,i);    
        double f_sum;
        f_sum=  sqrt( foot_for(0)*foot_for(0)+foot_for(1)*foot_for(1)+foot_for(2)*foot_for(2) );
        if(f_sum>=swing_contact_threadhold(i)){
            leg_swingphase_contact_est_count(i) ++;
            // contact_estimate_schedual(i)=0;   // 0 :即合力大于２，用０表示触碰
        }
        else{   
            leg_swingphase_contact_est_count(i) = 0;
            contact_estimate_schedual(i)=1;
        }

        if (leg_swingphase_contact_est_count(i) >= 3){
            contact_estimate_schedual(i)=0;   // 0 :即合力大于２，用０表示触碰
        }
    }

    //lcc 摆动触碰估计
    leg_swingphase_contact_est.setZero();
    for(int i=0;i<6;i++){       
        if(contact(i)==0 && contact_estimate_schedual(i)==0 && phase(i) >= 0.2 && phase(i) <= 0.9) {       
            leg_swingphase_contact_est(i)=1;   //表示摆动时触碰东西

            // std::cout<<" contact_estimate_schedual \n:"<< contact_estimate_schedual <<std::endl;
            // std::cout<<" _footEndForce \n:"<< _footEndForce <<std::endl;
            // std::cout<<" phase \n:"<< phase <<std::endl;
            // std::cout<<" contact \n:"<< contact <<std::endl;
            // std::cout<<" leg_swingphase_contact_est \n:"<< leg_swingphase_contact_est <<std::endl;
            // printf(" \n ");
            // exit(0);
        }
    }

    for (int i=0; i<6; ++i) {
        Eigen::Vector3d foot_for = _footEndForce.block<3,1>(0,i);    

        //法二：　直接通过合力来判断
        double f_sum;
        f_sum=  sqrt( foot_for(0)*foot_for(0)+foot_for(1)*foot_for(1)+foot_for(2)*foot_for(2) );
        if(f_sum>=supor_contact_threadhold)
            contact_estimate_schedual(i)=0;   // 0 :即合力大于２，用０表示触碰
        else
            contact_estimate_schedual(i)=1;
    }

    //lcc 20230519:触地估计
    for(int i=0;i<6;i++){       
            if(contact(i)==1 && contact_estimate_schedual(i)==0)//如果contact_estimate_schedual=0即表示触碰物体　&&　cpg=0即表示支撑态
            {       
                leg_suportingphase_contact_est(i)=1;   //1表示触地
                // std::cout<<" _footEndForce \n:"<< _footEndForce <<std::endl;

            } 
            else if(contact(i)==0)  //lcc 如果cpg处于摆动相, 只有到达摆动态，腿的触地信号才会重置
            {
                leg_suportingphase_contact_est(i)=0;   //0表示bu触地
            }
    }
}




