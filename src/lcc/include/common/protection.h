#ifndef PROTECTPRO_H
#define PROTECTPRO_H

#include <stdio.h>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>

class DataUnusualProtect
{
    private:
        double _datda_out[3]={0},_max_dia[3]={0},_rec_last[3]={0},_rec_datain[3]={0};
        Eigen::Vector3d _datda_out_;
        double _last[3]={0};
        int _enable[3]={0};
        int _first_enable[3]={0};
        double _RAD=3.1415926/180,_RAD2=180/3.141592;

        int vel_erro_printf_flag[3]={0};
        int pos_erro_printf_flag[3]={0};
    public:
        bool StopFlag=true;
        bool diff_val_flag=true;
        bool vel_lim_flag=true;

        /**
        * @brief 跟据电机实际角度和期望角度点差值来保护,如果两者差值大于设定的可容忍的差值，那么标志失败;对电机实际速度和位置进行限制保护
        * @param * des_pos  
        * @param * act_pos 
        * @param   tolerate_differ_v
        * @param * act_vel 
        * @param   vel_limit_v
        * @param   motor_number:  =3时，开启3个电机的保护; =0时，开启0号电机;  =1时，开启1号电机;  =2时，开启2号电机;
        * @author lcc
        */
        // void velLimAndDifFroDesPosAndActPos(char **leg_name,int motor_number,double * des_pos, double * act_pos,double tolerate_differ_v,double * act_vel, double vel_limit_v)
        void velLimAndDifFroDesPosAndActPos(int leg_number, int motor_number, Eigen::Vector3d des_pos_, Eigen::Vector3d act_pos_,double tolerate_differ_v, Eigen::Vector3d act_vel_, double vel_limit_v)
        {  
            double des_pos[3], act_pos[3], act_vel[3];
            for(int i=0; i<3; i++)
            {
                des_pos[i]=des_pos_(i);
                act_pos[i]=act_pos_(i);
                act_vel[i]=act_vel_(i);
            }

            if(motor_number==3)
            {
                for(int i=0;i<3;i++)
                {
                    if( fabs( des_pos[i]-act_pos[i] )>=tolerate_differ_v  || isnan(des_pos[i]) || isinf(des_pos[i]) || isnanf(des_pos[i]) || isinff(des_pos[i]))
                    {
                        diff_val_flag=false;
                        if(pos_erro_printf_flag[i]==0)
                        {
                            // std::cout<<*leg_name<<std::endl;
                            printf("velLimAndDifFroDesPosAndActPos leg_number:%d \n",leg_number);
                            printf(" diff_val_flag[%d]:%d  des_pos[%d]:%f   act_pos[%d]:%f   diff:%f   tolerate_v:%f   \n",
                                    i, diff_val_flag, i, des_pos[i]*_RAD2,i, act_pos[i]*_RAD2, 
                                    fabs( des_pos[i]-act_pos[i] )*_RAD2, tolerate_differ_v*_RAD2);       
                            pos_erro_printf_flag[i]=1;                 
                        }
                    }

                    if(   fabs(act_vel[i])>=vel_limit_v)
                    {
                        if(vel_erro_printf_flag[i]==0)
                        {
                            // std::cout<<*leg_name<<std::endl;
                            printf("velLimAndDifFroDesPosAndActPos leg_number:%d \n",leg_number);
                            printf("vel_lim_flag[%d]:%d  act_vel[%d]:%f  vel_limit_v:%f\n",
                                i,vel_lim_flag,i,act_vel[i],vel_limit_v); 
                            vel_erro_printf_flag[i]=1;                 
                        }
                        vel_lim_flag=false;
                    }
                } 
            }
            else if(motor_number==0)
            {
                int i=0;
                {
                    if( fabs( des_pos[i]-act_pos[i] )>=tolerate_differ_v  || isnan(des_pos[i]) || isinf(des_pos[i]) || isnanf(des_pos[i]) || isinff(des_pos[i]))
                    {
                        diff_val_flag=false;
                        // std::cout<<*leg_name<<std::endl;
                            printf("velLimAndDifFroDesPosAndActPos leg_number:%d \n",leg_number);
                        printf(" diff_val_flag[%d]:%d  des_pos[%d]:%f   act_pos[%d]:%f   diff:%f   tolerate_v:%f   \n",
                                i, diff_val_flag, i, des_pos[i]*_RAD2,i, act_pos[i]*_RAD2, 
                                fabs( des_pos[i]-act_pos[i] )*_RAD2, tolerate_differ_v*_RAD2);
                    }

                    if(   fabs(act_vel[i])>=vel_limit_v)
                    {
                        // std::cout<<*leg_name<<std::endl;
                            printf("velLimAndDifFroDesPosAndActPos leg_number:%d \n",leg_number);
                        printf(" vel_lim_flag[%d]:%d  act_vel[%d]:%f  vel_limit_v:%f\n",
                                i,vel_lim_flag,i,act_vel[i],vel_limit_v);
                        vel_lim_flag=false;
                    }

                } 
            }
            else if(motor_number==1)
            {
                int i=1;
                {
                    if( fabs( des_pos[i]-act_pos[i] )>=tolerate_differ_v  || isnan(des_pos[i]) || isinf(des_pos[i]) || isnanf(des_pos[i]) || isinff(des_pos[i]))
                    {
                        diff_val_flag=false;
                        // std::cout<<*leg_name<<std::endl;
                            printf("leg_number:%d \n",leg_number);
                        printf("velLimAndDifFroDesPosAndActPos diff_val_flag[%d]:%d  des_pos[%d]:%f   act_pos[%d]:%f   diff:%f   tolerate_v:%f   \n",
                                i, diff_val_flag, i, des_pos[i]*_RAD2,i, act_pos[i]*_RAD2, 
                                fabs( des_pos[i]-act_pos[i] )*_RAD2, tolerate_differ_v*_RAD2);
                    }

                    if(   fabs(act_vel[i])>=vel_limit_v)
                    {
                        // std::cout<<*leg_name<<std::endl;
                            printf("velLimAndDifFroDesPosAndActPos leg_number:%d \n",leg_number);
                        printf("vel_lim_flag[%d]:%d  act_vel[%d]:%f  vel_limit_v:%f\n",
                                i,vel_lim_flag,i,act_vel[i],vel_limit_v);
                        vel_lim_flag=false;
                    }

                } 
            }
            else if(motor_number==2)
            {
                int i=2;
                {
                    if( fabs( des_pos[i]-act_pos[i] )>=tolerate_differ_v  || isnan(des_pos[i]) || isinf(des_pos[i]) || isnanf(des_pos[i]) || isinff(des_pos[i]))
                    {
                        diff_val_flag=false;
                        // std::cout<<*leg_name<<std::endl;
                            printf("velLimAndDifFroDesPosAndActPos leg_number:%d \n",leg_number);
                        printf("diff_val_flag[%d]:%d  des_pos[%d]:%f   act_pos[%d]:%f   diff:%f   tolerate_v:%f   \n",
                                i, diff_val_flag, i, des_pos[i]*_RAD2,i, act_pos[i]*_RAD2, 
                                fabs( des_pos[i]-act_pos[i] )*_RAD2, tolerate_differ_v*_RAD2);
                    }

                    if(   fabs(act_vel[i])>=vel_limit_v)
                    {
                        // std::cout<<*leg_name<<std::endl;
                            printf("velLimAndDifFroDesPosAndActPos leg_number:%d \n",leg_number);
                        printf("vel_lim_flag[%d]:%d  act_vel[%d]:%f  vel_limit_v:%f\n",
                                i,vel_lim_flag,i,act_vel[i],vel_limit_v);
                        vel_lim_flag=false;
                    }

                } 
            }
        }

        void diff_val_flag_reset(void)
        {
            diff_val_flag = true;
            vel_lim_flag = true;
        }

        Eigen::Vector3d sendDataConPro(int leg_number, Eigen::Vector3d dataIn,double threshold)
        {   
            for(int i=0;i<3;i++)
            {
                // if(i==2)
                    // printf("i:%d StopFlag:%d _enable:%d threshold:%f _last:%f  dataIn:%f \n",
                    //         i,StopFlag,_enable[i],threshold*_RAD2,_last[i]*_RAD2,dataIn(i)*_RAD2);
                if( isnan(dataIn(i)) or isnanf(dataIn(i))  // 如果传进来的素据是 nan 或其他问题，那么 StopFlag=false -> 进入保护
                    or isinf(dataIn(i)) or isinff(dataIn(i)) or fabs(dataIn(i))>=3.14) StopFlag=false;

                if(_first_enable[i]==0)
                {
                    _last[i]=dataIn(i);
                    _first_enable[i]=1;
                    _datda_out_=dataIn;  //lcc 20230627: 保证第一次进来返回值不为0,直接把输入值当成返回值
                    // printf("_first_enable:%d  dataIn:%f _last:%f \n", _first_enable[i],dataIn(i),_last[i]);
                }
                else if(_enable[i]==0)
                {
                    if(fabs(_last[i]-dataIn(i))>=threshold)
                    {
                        _max_dia[i]=fabs(_last[i]-dataIn(i));
                        // std::cout<<*leg_name<<std::endl;
                            printf("sendDataConPro leg_number:%d \n",leg_number);
                        printf("i:%d StopFlag:%d _enable:%d threshold:%f _last:%f  dataIn:%f \n",
                            i,StopFlag,_enable[i],threshold*_RAD2,_last[i]*_RAD2,dataIn(i)*_RAD2);
                                printf("i:%d _max_dia:%f _rec_last:%f _rec_datain:%f _datda_out_:%f \n\n",
                            i, _max_dia[i]*_RAD2,_rec_last[i]*_RAD2,_rec_datain[i]*_RAD2,_datda_out_(i)*_RAD2);
                        _rec_last[i]=_last[i];
                        _rec_datain[i]=dataIn(i);



                        StopFlag=false;
                        _datda_out_(i)=_last[i];

                        printf("exit at sendDataConPro() !!!\n");

                        exit(0);

                    }
                    else
                    {
                        _datda_out_(i)=_last[i]=dataIn(i);
                        _enable[i]=1;                       
                    }
                }
                else if(StopFlag==true)
                {
                    if(fabs(_last[i]-dataIn(i))>=threshold)
                    {
                        _max_dia[i]=fabs(_last[i]-dataIn(i));
                        // std::cout<<*leg_name<<std::endl;
                            printf("sendDataConPro leg_number:%d \n",leg_number);
                        printf("i:%d StopFlag:%d _enable:%d threshold:%f _last:%f  dataIn:%f \n",
                            i,StopFlag,_enable[i],threshold*_RAD2,_last[i]*_RAD2,dataIn(i)*_RAD2);
                                printf("i:%d _max_dia:%f _rec_last:%f _rec_datain:%f _datda_out_:%f \n\n",
                            i, _max_dia[i]*_RAD2,_rec_last[i]*_RAD2,_rec_datain[i]*_RAD2,_datda_out_(i)*_RAD2);
                        _rec_last[i]=_last[i];
                        _rec_datain[i]=dataIn(i);

                        StopFlag=false;
                        _datda_out_(i)=_last[i];
                    }
                    else
                    {
                        _last[i]=dataIn(i);
                        _datda_out_(i)=dataIn(i);
                    }  
                }
                else
                {
                    _datda_out_(i)=_last[i];
                }
                // if(i==2)
                    // printf("i:%d _max_dia:%f _rec_last:%f _rec_datain:%f _datda_out_:%f \n\n",
                    //         i, _max_dia[i]*_RAD2,_rec_last[i]*_RAD2,_rec_datain[i]*_RAD2,_datda_out_(i)*_RAD2);
            }
            return _datda_out_;
        }

        bool RetFlag(void)
        {
            return StopFlag;
        }
 
};

#endif
