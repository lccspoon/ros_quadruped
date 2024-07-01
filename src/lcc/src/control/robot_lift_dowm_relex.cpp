#include"control/robot_lift_dowm_relex.h"
#include"control/neural_bezier_curve.h"
#include "FSM/State_PosReflex.h"
// #include <algorithm> 

simple_scheduler _lift_simpleScheduler[6];
simple_scheduler _dowm_simpleScheduler[6];
LagrangeInterpolator _LagrangeInterpolator[6];

// Eigen::Matrix<double,1,6>  _lowState->userFunctionMode.LEG_LIFT_TRIGGER;//20230907cheet
// Eigen::Matrix<double,1,6>  _lowState->userFunctionMode.LEG_DOWM_TRIGGER;//20230907cheet
// Eigen::Matrix<double,1,6>  LEG_MKAN_TRIGGER;//20230907cheet
// int  _lowState->userFunctionMode.life_reaction_off_on=0,_lowState->userFunctionMode.dowm_reaction_off_on=0,_lowState->userFunctionMode.mkan_reaction_off_on=0,berzier_shape_off_on = 0;
// int life_reaction_off_on_flag=0,dowm_reaction_off_on_flag=0,mkan_reaction_off_on_flag=0, berzier_shape_off_on_flag = 0;

Eigen::Matrix<double,1,6>  swing_touch_leg_number;//
Eigen::Matrix<double,1,6>  suporting_slip_leg_number;//
// Eigen::Matrix<double,3,6>  foot_lift_traj;
// Eigen::Matrix<double,3,6>  foot_dowm_traj;

int down_stage_switching_flag=0;
int down_stage_count=0;
linear_trans foot_dowmward_traj_res[6],foot_ditch_deepth_est_res[6],foot_dowm_traj_res[6];

// int lift_stage_flag=0;
int lift_init_flag=0;
Eigen::Matrix<double,1,6> lift_stage_switching_flag;
int lift_stage_count=0;
int changeBezierShape_flag[6]={0};
linear_trans foot_cross_traj_res[6], foot_lift_traj_res[6], foot_cross_object_est_res[6];

Eigen::Matrix<double,1,6> no_need_changing_dowm_traj, no_need_changing_lift_traj;

Eigen::Matrix<int,1,6>  lift_cpg_phase_last;
Eigen::Matrix<double,1,6>  lift_x_traj_rec;

Eigen::Matrix<double,3,6> rec_foot_lift_pos;
Eigen::Matrix<double,1,6> rec_foot_lift_pos_flag;
Eigen::Matrix<double,1,6> reset_rec_foot_lift_pos_flag;
Eigen::Matrix<double,3,6> rec_foot_cross_traj;
Eigen::Matrix<double,3,6> rec_foot_swing_traj;
Eigen::Matrix<double,1,6> enter_lift_count;
Eigen::Matrix<double,1,6> rec_cpg_period_number;
linear_trans reset_cross_traj_X[6];

void State_PosReflex::liftFollowReaction(void)
{
    //----  以下是抬升反应 ----//
    for (int j = 0; j < 2; j++)//！ 只有lf和rf进行自主抬升
    {   
        int i=0;
        // i=j*3;//！ 只有lf和rf进行自主抬升
        i=j;//！ 只有lf和rf进行自主抬升

        //! 首先进行触碰监测
        if(_contactEst.leg_swingphase_contact_est(i)==1 || _lowState->userFunctionMode.LEG_LIFT_TRIGGER(i)==1)  //lcc   lf
        // if( _lowState->userFunctionMode.LEG_LIFT_TRIGGER(i)==1)  //lcc   lf
        {
            swing_touch_leg_number(i)=1;
            // rec_foot_swing_traj(0,i)=foot_swing_traj(0,i);
            rec_cpg_period_number(i)=cpg_period_count(i);
            // std::exit(0);

            // printf("\n------inininin------\n");
        }

        //！然后，启动 贝赛尔 曲线的 形状调节，帮助抬高轨迹
        // 如果腿在摆动中触碰，并且　腿抬高轨迹foot_cross_traj　超过了xxcm,那么开始考虑改变足端摆动轨迹
        // if( swing_touch_leg_number(i)==1 && fabs( foot_cross_traj(2,i) )>1*0.01)  
        // {
        //     // changeBezierShape_flag[i]=1;
        // }
        // changeBezierShape(i);//！ 如果：changeBezierShape_flag[i]=1，那么第i条腿开始改变轨迹形状参数

        //！ 抬升轨迹foot_cross_traj是由原摆动轨迹和每一小段上升的保弧形bezier轨迹组成 
        // 足端抬升的轨迹，和上面的改变原摆动态轨迹的形状参数不同
        // std::cout<<" neur_bezier[i].lamdaX"<<std::endl;
        // std::cout<< neur_bezier[i].lamdaX<<std::endl;
        neur_bezier_lift_curve[i].Set_PX<< 0 , -1 , -3 , -5 , -7 , -5 , -3 , -1 , -0.5;  //输入单位cm 设置态升轨迹
        // neur_bezier_lift_curve[i].Set_PX<< 0 , -1 , -3 , -10 , -14 , -10 , -3 , -1 , -0.5;  //输入单位cm 设置态升轨迹
        neur_bezier_lift_curve[i].Set_PY<< -3 + 3,-2.5 + 3, -2.3 + 3, -0.5 + 3, 1 + 3, 2.7 + 3, 3 + 3, 4.0 + 3, 3.75 + 3;  //输入单位cm　设置态升轨迹*0.01 + 3*0.01, 2.7*0.01 + 3*0.01, 3*0.01 + 3*0.01, 4.0*0.01 + 3*0.01, 3.75*0.01 + 3*0.01;  //输入单位cm　设置态升轨迹
        // #if HARD_WARE==1
        //     neur_bezier_lift_curve[i].Set_PX=neur_bezier_lift_curve[i].Set_PX*0.01  *0.15  *0.25;   // *0.15 -> 轨迹缩小10倍,　
        //     // neur_bezier_lift_curve[i].Set_PX=neur_bezier_lift_curve[i].Set_PX*0.01  *0.15  *0.1;   // *0.15 -> 轨迹缩小10倍,　
        //     neur_bezier_lift_curve[i].Set_PY=neur_bezier_lift_curve[i].Set_PY*0.01  *0.05  *2;  // *0.05-> 大概抬高0.4cm
        // #elif HARD_WARE==2
            neur_bezier_lift_curve[i].Set_PX=neur_bezier_lift_curve[i].Set_PX*0.01  *0.15  *0.25 *1;   // *0.15 -> 轨迹缩小10倍,　
            neur_bezier_lift_curve[i].Set_PY=neur_bezier_lift_curve[i].Set_PY*0.01  *0.05  *2 *1;  // *0.05-> 大概抬高0.4cm
        // #endif

        //！ 开始变轨迹，我们最终要得到的这： foot_cross_traj
        //!  若　lift_stage_switching_flag==1　表示轨迹正在复原，此时不可调整轨迹
        if( swing_touch_leg_number(i)==1  && lift_stage_switching_flag(i)==0 && (*_contact_hex)(i)==0) 
        // if( (*_contact_hex)(i)==0 && lift_cpg_phase_last(i)==1 && _lowState->userFunctionMode.LEG_LIFT_TRIGGER(i)==1) 
        // if( _lowState->userFunctionMode.LEG_LIFT_TRIGGER(i)==1) 
        // if( (*_contact_hex)(i)==0 && _lowState->userFunctionMode.LEG_LIFT_TRIGGER(i)==1) 
        {  
            // std::cout<<" LEG_LIFT_TRIGGER \n:"<<_lowState->userFunctionMode.LEG_LIFT_TRIGGER<<std::endl;
            // printf("LF_TRIGGER:%d   ",LF_TRIGGER);
            // printf("RF_TRIGGER:%d\n",RF_TRIGGER);
            double ttt;
            ttt=-1.0+_lift_simpleScheduler[i].retSimpleScheduler(1, 0.01)* 5.0;

            #if HARD_WARE==1
                //! 出发反应后，阈值降低，让避障更灵敏
                swing_contact_threadhold(i)=5;//lcc 20230626 //lcc 20240627 屏蔽
            #endif

            //停止cpg来越过障碍
            _Cpg.cpg_stop_flag=1;

            //！ foot_cross_traj最终叠加在foot_trajectory上：
            //！  -> foot_lift_traj是foot_cross_traj的累加项，neur_bezier_lift_curve是每一小段上升曲线
            foot_cross_traj.block<3,1>(0,i)= foot_lift_traj.block<3,1>(0,i)+
                                neur_bezier_lift_curve[i].bezierCurve( ttt , 1);
            if( _lowState->userFunctionMode.mkan_reaction_off_on==1 )
                _lowState->userFunctionMode.LEG_DOWM_TRIGGER(i)=1;//cheet

            // printf(" ininin\n");

            if( ttt>=0.9 )  // 因为大于0.9后，轨迹会返回-nan,也不知道为什么
            {   
                //！ 这个变量用于累加腿的抬升高度
                foot_lift_traj.block<3,1>(0,i)=foot_cross_traj.block<3,1>(0,i);  
                //！　记录腿的抬升高度并且用这个高度来估计腿遇到的障碍物．
                foot_cross_object_est(i)=foot_lift_traj(2,i);  
                //！ 记录叠加了多少次的neur_bezier_lift_curve。用于输出打印
                enter_lift_count(i)=enter_lift_count(i)+1;

                _lift_simpleScheduler[i].reSet();
                swing_touch_leg_number(i)=0;
                _Cpg.cpg_stop_flag=0;

                // 一个标志位，用于记录已经出发抬升，让后四条腿跟上
                rec_foot_lift_pos_flag(i)=1;   // 目前是手动控制的，没用上
            }
        }

        //lcc 20240627 屏蔽
        #if HARD_WARE==1
            //！ 恢复初始阈值
            if((*_contact_hex)(i)==1)
                swing_contact_threadhold(i)=14;//lcc 20230626
        #endif

            for (int j = 0; j < 2; j++)
            {   
                int i=0;
                i=j*1;

                // 如果前两腿其中的一腿 处于摆动态度 且 抬升高度大于1cm 且 距离抬升已经超过一个cpg周期
                if((*_contact_hex)(i)==0 && fabs( foot_cross_traj(2,i) )>1*0.01 && ( cpg_period_count(i)-rec_cpg_period_number(i) ) >=2 )
                {
                    int tttt=int(1/_Cpg.control_cycle)*0.5*0.75;
                    foot_cross_traj(0,i) = reset_cross_traj_X[i].linearConvert(foot_cross_traj(0,i),0, tttt);
                }
            }
    }
    liftFollowTraject(); //后面的4条腿跟随前面两条腿的轨迹来越障
}

void State_PosReflex::changeBezierShape(int i){}

Eigen::Matrix<double,1,6> leg_follow_cross_traj_flag;
void State_PosReflex::liftFollowTraject(void)   // 后四条腿跟随前腿的轨迹
{
    // for (int j = 0; j < 2; j++)  // 获得lf rf越障时的起始足端位置　　
    // {   
    //     int i=0;
    //     i=j*3;
    //     // 如果已经进入了轨迹抬升阶段rec_foot_pos_flag(i)==1 ，　且如果当前是支撑态cpg_touch_down_scheduler(i)==0，并且　腿抬高轨迹 foot_cross_traj-rec_foot_cross_traj　超过了xxcm
    //     if( rec_foot_lift_pos_flag(i)==1 && (*_contact_hex)(i)==0 && 
    //         fabs( fabs( foot_cross_traj(2,i) ) - fabs( rec_foot_cross_traj(2,i) ) ) >1*0.01) // rec_foot_cross_traj是一个用来记录foot_cross_traj的变量，保证可以让他们两个的差值大于１时才可以进入if
    //     {
    //         // 一个cpg周期包含摆动态和支撑态;假设摆动或支撑态步长为3cm,则一个周期机身前进６cm;支撑态时，腿位置不变，摆动态时，腿向前跨６cm;
    //         // 所以，每一个摆动态，足端向前６cm,即ｌeg_root.step_set_length(i)
    //         rec_foot_lift_pos(0,i)=world_root.foot_des_pos(0,i)- step_set_length(i); // 减去步长，相当于记录开始进入摆动态时的位置 
    //         rec_foot_lift_pos(1,i)=world_root.foot_des_pos(1,i);
    //         rec_foot_lift_pos(2,i)=world_root.foot_des_pos(2,i);

    //         reset_rec_foot_lift_pos_flag(i)=1;
    //         rec_foot_cross_traj=foot_cross_traj; // rec_foot_cross_traj是一个用来记录foot_cross_traj的变量，保证可以让他们两个的差值大于１时才可以进入if
    //         // printf("%d aaa\n",i);
    //         // std::cout<<" rec_foot_lift_pos "<<std::endl;
    //         // std::cout<< rec_foot_lift_pos <<std::endl;
    //         // exit(0);
    //     }
    //     else if((*_contact_hex)(i)==1 && reset_rec_foot_lift_pos_flag(i)==1)
    //     {
    //         rec_foot_lift_pos_flag(i)==0;   
    //         // printf("%d bbb\n",i);
    //         // exit(0);
    //     }
    // }

    // std::cout<<" _lowState->userFunctionMode.LEG_LIFT_TRIGGER: "<< _lowState->userFunctionMode.LEG_LIFT_TRIGGER <<std::endl;
    // std::cout<<" _lowState->userFunctionMode.mkan_reaction_off_on: "<< _lowState->userFunctionMode.mkan_reaction_off_on <<std::endl;

    // if( rec_foot_lift_pos(0,0)!=0 ) // 如果记录到lf有变轨迹的起始位置，那么lm lb到这个起始位置也需要变轨迹来越障
    {
        if
        (
        // ( fabs( fabs( world_root.foot_des_pos(0, 1) ) - fabs( rec_foot_lift_pos(0 ,0) ) ) < 4*0.01 &&
        //     (*_contact_hex)(1)==1 && lift_cpg_phase_last(1)==0 )   
        // || 
        ((*_contact_hex)(3)==0 && lift_cpg_phase_last(3)==1 && _lowState->userFunctionMode.LEG_LIFT_TRIGGER(3)==1)//！cheet
        )
        {
            //！ 目地是得到这个flag
            leg_follow_cross_traj_flag(0,3)=1;
            //！ 目地是得到这个flag
            // changeBezierShape_flag[1]=1;
            _lowState->userFunctionMode.LEG_LIFT_TRIGGER(3)=0;
            printf(" lm变轨迹\n");
            // if( _lowState->userFunctionMode.mkan_reaction_off_on==1 )
            // _lowState->userFunctionMode.LEG_DOWM_TRIGGER(3)=1;//cheet
            // exit(0);
        }

        if(
            // ( fabs( fabs( world_root.foot_des_pos(0, 2) ) - fabs( rec_foot_lift_pos(0 ,0) ) ) < 4*0.01 &&
            // (*_contact_hex)(2)==1 && lift_cpg_phase_last(2)==0 )
            // || 
            ((*_contact_hex)(5)==0 && lift_cpg_phase_last(5)==1 && _lowState->userFunctionMode.LEG_LIFT_TRIGGER(5)==1)//！cheet
            )
        {
            printf(" lb变轨迹\n");
            // if( _lowState->userFunctionMode.mkan_reaction_off_on==1 )
            // _lowState->userFunctionMode.LEG_DOWM_TRIGGER(5)=1;//cheet
            // std::cout<<" leg_follow_cross_traj_flag "<<std::endl;
            // std::cout<< leg_follow_cross_traj_flag <<std::endl;
            // std::cout<<" _lowState->userFunctionMode.LEG_LIFT_TRIGGER "<<std::endl;
            // std::cout<< _lowState->userFunctionMode.LEG_LIFT_TRIGGER <<std::endl;

            leg_follow_cross_traj_flag(0,5)=1;
            // changeBezierShape_flag[2]=1;
            _lowState->userFunctionMode.LEG_LIFT_TRIGGER(5)=0;
            // exit(0);
        }
    }
    // if( rec_foot_lift_pos(0,3)!=0 ) // 如果记录到rf有变轨迹的起始位置，那么rm rb到这个起始位置也需要变轨迹来越障
    {
        if(
            // ( fabs( fabs( world_root.foot_des_pos(0, 4) ) - fabs( rec_foot_lift_pos(0 ,3) ) ) < 4*0.01 &&
            // (*_contact_hex)(4)==1 && lift_cpg_phase_last(4)==0 )
            // || 
            ((*_contact_hex)(2)==0 && lift_cpg_phase_last(2)==1 && _lowState->userFunctionMode.LEG_LIFT_TRIGGER(2)==1)//cheet
        )
        {
            printf(" rm变轨迹\n");
            // if( _lowState->userFunctionMode.mkan_reaction_off_on==1 )
            // _lowState->userFunctionMode.LEG_DOWM_TRIGGER(2)=1;//cheet
            // std::cout<<" leg_follow_cross_traj_flag "<<std::endl;
            // std::cout<< leg_follow_cross_traj_flag <<std::endl;
            // std::cout<<" _lowState->userFunctionMode.LEG_LIFT_TRIGGER "<<std::endl;
            // std::cout<< _lowState->userFunctionMode.LEG_LIFT_TRIGGER <<std::endl;

            leg_follow_cross_traj_flag(0,2)=1;
            // changeBezierShape_flag[4]=1;
            _lowState->userFunctionMode.LEG_LIFT_TRIGGER(2)=0;
            // exit(0);
        }

        if(
            // ( fabs( fabs( world_root.foot_des_pos(0, 5) ) - fabs( rec_foot_lift_pos(0 ,3) ) ) < 4*0.01 &&
            // (*_contact_hex)(5)==1 && lift_cpg_phase_last(5)==0 )
            // || 
            ((*_contact_hex)(4)==0 && lift_cpg_phase_last(4)==1 && _lowState->userFunctionMode.LEG_LIFT_TRIGGER(4)==1)//cheet
        )
        {
            leg_follow_cross_traj_flag(0,4)=1;
            // changeBezierShape_flag[5]=1;
            _lowState->userFunctionMode.LEG_LIFT_TRIGGER(4)=0;
            printf(" rb变轨迹\n");
            // if( _lowState->userFunctionMode.mkan_reaction_off_on==1 )
            // _lowState->userFunctionMode.LEG_DOWM_TRIGGER(4)=1;//cheet
            // exit(0);
        }
    }

    // _lowState->userFunctionMode.LEG_LIFT_TRIGGER.setZero();
    // std::cout<<" _lowState->userFunctionMode.LEG_LIFT_TRIGGER: "<< _lowState->userFunctionMode.LEG_LIFT_TRIGGER <<std::endl;

    for(int i=0; i<6; i++)
    {
        // changeBezierShape(i);  //这个函数里面的不同腿的变现是由changeBezierShape_flag判断后执行的
        lift_cpg_phase_last(i)=(*_contact_hex)(i);
    }

    int tt=int(1/_Cpg.control_cycle)*0.5*0.15;
    if(leg_follow_cross_traj_flag(0,3)==1 )
    {
        foot_lift_traj(0,1)=0;//lcc 20230627
        foot_lift_traj.block<3,1>(0,3)=foot_cross_traj_res[3].linearConvert(foot_lift_traj.block<3,1>(0,3),foot_lift_traj.block<3,1>(0,1), tt);
        foot_cross_traj.block<3,1>(0,3) = foot_lift_traj.block<3,1>(0,3);
    }
    else if( foot_cross_traj_res[3].retConvDoneFlag()==true )
        leg_follow_cross_traj_flag(0,3)=0;

    if(leg_follow_cross_traj_flag(0,5)==1 )
    {
        foot_lift_traj(0,1)=0;//lcc 20230627
        foot_lift_traj.block<3,1>(0,5)=foot_cross_traj_res[5].linearConvert(foot_lift_traj.block<3,1>(0,5),foot_lift_traj.block<3,1>(0,1), tt);
        foot_cross_traj.block<3,1>(0,5) = foot_lift_traj.block<3,1>(0,5);
    }
    else if( foot_cross_traj_res[5].retConvDoneFlag()==true )
        leg_follow_cross_traj_flag(0,5)=0;

    if(leg_follow_cross_traj_flag(0,2)==1 )
    {
        foot_lift_traj(0,0)=0;//lcc 20230627
        foot_lift_traj.block<3,1>(0,2)=foot_cross_traj_res[2].linearConvert(foot_lift_traj.block<3,1>(0,2),foot_lift_traj.block<3,1>(0,0), tt);
        foot_cross_traj.block<3,1>(0,2) = foot_lift_traj.block<3,1>(0,2);
    }
    else if( foot_cross_traj_res[2].retConvDoneFlag()==true )
        leg_follow_cross_traj_flag(0,2)=0;

    if(leg_follow_cross_traj_flag(0,4)==1 )
    {
        foot_lift_traj(0,0)=0;//lcc 20230627
        foot_lift_traj.block<3,1>(0,4)=foot_cross_traj_res[4].linearConvert(foot_lift_traj.block<3,1>(0,4),foot_lift_traj.block<3,1>(0,0), tt);
        foot_cross_traj.block<3,1>(0,4) = foot_lift_traj.block<3,1>(0,4);
    }
    else if( foot_cross_traj_res[4].retConvDoneFlag()==true )
        leg_follow_cross_traj_flag(0,4)=0;

}

Eigen::Matrix<double,1,6>  dowm_cpg_phase_last;
Eigen::Matrix<double,1,6>  dowm_x_traj_rec;
void State_PosReflex::dowmwardReaction(void)
{
    //----  以下是下探反应 ----//
    for (int i = 0; i < 6; i++)
    {
        // Eigen::Vector3d temp_x;
        // temp_x=foot_swing_traj.block<3,1>(0,i);
        // if( (*_contact_hex)(i)==1 &&  dowm_cpg_phase_last(i)==0 ) // 如果当前是支撑态　且上一时刻是摆动态
        // {
        //     dowm_x_traj_rec(i)=temp_x(0);  // 记录进入支撑态那一时刻，ｘ方向的轨迹
        // }
        dowm_cpg_phase_last(i)=(*_contact_hex)(i);

        //! 如果cpg支撑态 && leg_suportingphase_contact_est不触地 
        // if( _contactEst.leg_suportingphase_contact_est(i)==0 && (*_contact_hex)(i)==1 )
        if( _lowState->userFunctionMode.LEG_DOWM_TRIGGER(i)==1 && (*_contact_hex)(i)==1 )
        {
            // if( fabs( dowm_x_traj_rec(i)- temp_x(0) ) >=0.05 *0.01 && fabs( dowm_x_traj_rec(i))>=1 *0.01 ) //如果在支撑态下，轨迹前进了　0.05 厘米 && 记录的轨迹必需大于１cm
            // if( (*_phase_hex)(i)>=0.05 && (*_phase_hex)(i)<=0.2) //如果在支撑态下，轨迹前进了　0.05 厘米 && 记录的轨迹必需大于１cm
            if( (*_phase_hex)(i)>=0.0001 && (*_phase_hex)(i)<=0.2) //如果在支撑态下，轨迹前进了　0.05 厘米 && 记录的轨迹必需大于１cm
            {
                if( _lowState->userFunctionMode.mkan_reaction_off_on==1 )
                {
                    if(_lowState->userFunctionMode.LEG_DOWM_TRIGGER(i)==1 )//_lowState->userFunctionMode.LEG_DOWM_TRIGGER(i)==1是cheet的
                    {
                        suporting_slip_leg_number(i)=1;
                        // printf("leg(%d) touch dowm silp!!\n ",i);
                        // exit(0);
                    }
                }
                else
                {
                    suporting_slip_leg_number(i)=1;
                    // printf("leg(%d) touch dowm silp!!\n ",i);
                }
            }
        }

        if(suporting_slip_leg_number(i)==1 && down_stage_switching_flag==0 && (*_contact_hex)(i)==1 ) 
        {   
            _Cpg.cpg_stop_flag=1;
            foot_dowmward_traj.block<3,1>(0,i)= foot_dowm_traj.block<3,1>(0,i)+
                                    _LagrangeInterpolator[i].liftLegOne_cm(0.00001, -0.25, _dowm_simpleScheduler[i].retSimpleScheduler(1, 0.1) );

            // printf(" inininin\n ");

            if(_dowm_simpleScheduler[i].retTime()==1)
            {   
                foot_dowm_traj.block<3,1>(0,i)=foot_dowmward_traj.block<3,1>(0,i);  
                foot_ditch_deepth_est(i)=foot_dowm_traj(2,i);  

                _dowm_simpleScheduler[i].reSet();
                suporting_slip_leg_number(i)=0;
                _Cpg.cpg_stop_flag=0;
            }
        }
    }
}

void State_PosReflex::berzierShapeChangeRecation(void){}

int set_z_deviation_reset_flag=0;
void State_PosReflex::adaptive_control(void)
{
    // if( berzier_shape_off_on == 1) berzierShapeChangeRecation();
    //  _lowState->userFunctionMode.life_reaction_off_on = 1;
    if( _lowState->userFunctionMode.life_reaction_off_on==1) liftFollowReaction();
    else
    {
        Eigen::Vector3d Zero;
        Zero.setZero();
        set_z_deviation_reset_flag = 1;
        for(int j=0; j<6; j++)
        {
            int i=0;
            i=j;

            _lowState->userFunctionMode.LEG_LIFT_TRIGGER(i) = 0;
            leg_follow_cross_traj_flag(i) = 0;

            lift_stage_switching_flag(i)=1;
            _lift_simpleScheduler[i].reSet();
            swing_touch_leg_number(i)=0;
            _Cpg.cpg_stop_flag=0;

            Eigen::Vector3d Zero;
            Zero.setZero();
            int tt=int(1/_Cpg.control_cycle)*0.2;
            foot_cross_traj.block<3,1>(0,i)=
                        foot_cross_traj_res[i].linearConvert(foot_cross_traj.block<3,1>(0,i),Zero, tt);
            foot_lift_traj.block<3,1>(0,i)=
                        foot_lift_traj_res[i].linearConvert(foot_lift_traj.block<3,1>(0,i), Zero, tt);
            foot_cross_object_est(i)=foot_cross_object_est_res[i].linearConvert(foot_cross_object_est(i), 0, tt);

            if( foot_cross_traj_res[i].retConvDoneFlag()==true && foot_cross_traj_res[i].retConvDoneFlag()==true 
            && foot_cross_object_est_res[i].retConvDoneFlag()==true  )
            {
                lift_stage_switching_flag(i)=0;
            }

            if( foot_cross_traj_res[i].retConvDoneFlag()==true && foot_cross_traj_res[i].retConvDoneFlag()==true  )
            {
                lift_stage_switching_flag(i)=0;
                // lift_stage_flag=0;
            }
        }
    }

    if(_lowState->userFunctionMode.dowm_reaction_off_on==1) dowmwardReaction();
    else
    {
        Eigen::Vector3d Zero;
        Zero.setZero();
        for(int i=0; i<6; i++)
        {
            _lowState->userFunctionMode.LEG_DOWM_TRIGGER(i)=0;
            suporting_slip_leg_number(i)=0; 
            _dowm_simpleScheduler[i].reSet();

            down_stage_switching_flag=1;
            int tt=int(1/_Cpg.control_cycle)*0.2;
            foot_dowmward_traj.block<3,1>(0,i)=
                        foot_dowmward_traj_res[i].linearConvert(foot_dowmward_traj.block<3,1>(0,i),Zero, tt);

            foot_dowm_traj.block<3,1>(0,i)=
                        foot_dowm_traj_res[i].linearConvert(foot_dowm_traj.block<3,1>(0,i), Zero, tt);

            foot_ditch_deepth_est(i)=foot_ditch_deepth_est_res[i].linearConvert(foot_ditch_deepth_est(i), 0, tt);

            if( foot_dowmward_traj_res[0].retConvDoneFlag()==true && foot_dowmward_traj_res[3].retConvDoneFlag()==true
            && foot_ditch_deepth_est_res[i].retConvDoneFlag()==true  )
            {
                down_stage_switching_flag=0;
            }
        }
    }

    for (int i = 0; i < 6; i++)
    {
        // foot_trajectory.block<3,1>(0,i)=foot_cross_traj.block<3,1>(0,i)+foot_dowmward_traj.block<3,1>(0,i)+foot_swing_traj.block<3,1>(0,i); 
        foot_trajectory.block<3,1>(0,i)=foot_cross_traj.block<3,1>(0,i)+foot_dowmward_traj.block<3,1>(0,i); 
    }
    // foot_trajectory(2) = 0.027;

    // 以下是机身z方向高度调节和高度复位
    if(set_z_deviation_reset_flag==1) // 让所有腿的　foot_cross_traj　都变回0
    {   
        int tt=int(1/_Cpg.control_cycle)*0.2;
        set_z_deviation_adaptiv=deviation_conver_z_adaptive.linearConvert(set_z_deviation_adaptiv, 0, tt);

        if(deviation_conver[2].retConvDoneFlag()==true)
            set_z_deviation_reset_flag=0;
        // printf("\n set_z_deviation_reset_flag==1111111111111111111111111 \n");
    }
    else
    {
        if( _lowState->userFunctionMode.life_reaction_off_on==1 || _lowState->userFunctionMode.dowm_reaction_off_on==1)
        {
            Eigen::Matrix<double, 1, 6> temp_hight, temp_deepth;
            temp_hight=foot_cross_hight_sort.sort(foot_cross_object_est);  //lcc 20230519:通过冒泡排序，将跨越高度提出来，将来用做机身高度调节
            temp_deepth=foot_ditch_deepth_sort.sort(foot_ditch_deepth_est);  
            set_z_deviation_adaptiv = deviation_conver_z_adaptive.linearConvert(set_z_deviation_adaptiv, temp_hight(5)/1.0, 1) * 2;
            // printf("\n set_z_deviation_reset_flag==00000000000000000000000000 \n");
        }
    }

    // std::cout << "foot_cross_traj\n" << foot_cross_traj<<std::endl;
    // std::cout << "foot_cross_object_est\n" << foot_cross_object_est<<std::endl;

    // std::cout << "foot_dowmward_traj\n" << foot_dowmward_traj<<std::endl;
    // std::cout << "foot_cross_object_est\n" << foot_cross_object_est<<std::endl;
}

