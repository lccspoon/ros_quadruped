#ifndef _NEURALBEZIERCURVE_H
#define _NEURALBEZIERCURVE_H

#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <stdio.h>
using namespace Eigen;

class BubbleSort 
{

    private:
    double data_record=0;
    int count=0;

    public:

    MatrixXd sort(MatrixXd matrix) //lcc  传入一个固定维度的矩阵，返回排序后的矩阵
    {
        int rows = matrix.rows();
        int cols = matrix.cols();

        for (int i = 0; i < rows; ++i) {
            
            
            for (int j = 0; j < cols - 1; ++j) {
                for (int k = 0; k < cols - j - 1; ++k) {
                    if (matrix(i, k) > matrix(i, k + 1)) {
                        std::swap(matrix(i, k), matrix(i, k + 1));


                        
                    }
                }
            }
        }
        return matrix;
    }

    double sort_continuet(double data_in)  //lcc 一直传入数据，返回最大的数
    {

        // if( fabs(data_record-data_in)<=0.1 * 0.01 ) 
        // {
        //     data_record=data_record;
        //     // count++;
        // }
        // else 
        if(data_record>data_in) 
        {
            data_record=data_record;
            // count=0;
        }
        else
        {
            data_record=data_in;
            // count=0;
        }
        // printf("count:%d\n",count);
        return data_record;       
    }

    // int ret_sort_continuet_max_flag(void)
    // {
    //     int fff=0;
    //     if(count>=20)
    //     {
    //         fff=1;
    //     }    
    //     return fff;
    // }

};

class LagrangeInterpolator 
{
    private:
        VectorXd x_;
        VectorXd y_;
        int n_;

    public:
        void setX(const VectorXd& x) 
        {
            x_ = x;
            n_ = x.size();
        }

        void setY(const VectorXd& y) 
        {
            y_ = y;
        }

        VectorXd interpolate(const VectorXd& xi) 
        {
            MatrixXd L = MatrixXd::Ones(xi.size(), n_);

            for (int j = 0; j < n_; j++) {
                for (int i = 0; i < n_; i++) {
                    if (i != j) {
                        L.col(j) = L.col(j).array() * (xi.array() - x_(i)) / (x_(j) - x_(i));
                    }
                }
            }

            VectorXd yi = L * y_;
            return yi;
        }

        void eg(void)
        {
            VectorXd x(9), y(9);
            x << 0, 1, 2, 3, 4, 5, 6, 7, 8;
            y << 0, 20, 30, 40, 35, 25, 16, 10, 0;

            int numPoints = 1000;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            for (int i = 0; i < numPoints; i++) 
            {
                std::cout << "(" << xi(i) << ", " << yi(i) << ")" << std::endl;
            }
        }

        void eg_bytime(double dt)  // dt: 0~1;
        {
            VectorXd x(9), y(9);
            x << 0, 1, 2, 3, 4, 5, 6, 7, 8;
            y << 0, 20, 30, 40, 35, 25, 16, 10, 0;

            int numPoints = 200;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            if(dt<=0) dt=0;
            if(dt>=1) dt=1;
            std::cout << "(" << xi( std::round( (numPoints-1)*dt) ) << ", " << yi( std::round( (numPoints-1)*dt) ) << ")"<< dt << std::endl;
        }

        Vector3d liftLegOne_cm(double _x, double _y, double dt)  // dt: 0~1;
        {
            VectorXd x(2), y(2);
            x << 0, _x*0.01;
            y << 0, _y*0.01;

            int numPoints = 100;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            if(dt<=0) dt=0;
            if(dt>=1) dt=1;
            // std::cout << "(" << xi( std::round( (numPoints-1)*dt) ) << ", " << yi( std::round( (numPoints-1)*dt) ) << ")"<< dt << std::endl;

            Vector3d retPos;
            retPos<< xi( std::round( (numPoints-1)*dt) ), 0, yi( std::round( (numPoints-1)*dt) );

            // std::cout<<"retPos"<< std::endl;
            // std::cout<<retPos.transpose()<< std::endl;

            return retPos;
        }

        int _y_higher=1;
        Vector3d legLiftAccount_cm(double _x, double _y, double dt)  // dt: 0~1;
        {
            VectorXd x(2), y(2);
            x << 0, _x*0.01;

            if(dt==1)
                _y_higher++;

            y << 0, _y_higher * _y*0.01;


            int numPoints = 100;
            VectorXd xi = VectorXd::LinSpaced(numPoints, x(0), x(x.size() - 1));

            setX(x);
            setY(y);
            VectorXd yi = interpolate(xi);

            // 输出插值点的坐标
            if(dt<=0) dt=0;
            if(dt>=1) dt=1;
            // std::cout << "(" << xi( std::round( (numPoints-1)*dt) ) << ", " << yi( std::round( (numPoints-1)*dt) ) << ")"<< dt << std::endl;

            Vector3d retPos;
            retPos<< xi( std::round( (numPoints-1)*dt) ), 0, yi( std::round( (numPoints-1)*dt) );

            return retPos;
        }

};


class simple_scheduler
{
    private:
        double ret_t=0;
        // linear_trans a;
        int flag1=0;
        int flag_stop=0;
    public:
        double retSimpleScheduler(double T=1, double dt=0.01)   //返回 0~T
        {
            if(flag_stop==0)
            {
                ret_t=ret_t+dt;

                if(ret_t>=T && flag1==0)
                {
                    ret_t=T;
                    flag1=1;
                }
                else if(flag1==1)
                {
                    ret_t=0;
                    flag1=0;
                }  
            }
            return ret_t;
        }

        double  retTime(void)
        {
            return ret_t;
        }

        void  stopTime(void)
        {
            flag_stop=1;
        }

        void reSet(void)
        {
            ret_t=0;
            flag_stop=0;
        }
        // simple_scheduler(/* args */);
        // ~simple_scheduler();

};

// simple_scheduler::simple_scheduler(/* args */)
// {
// }

// simple_scheduler::~simple_scheduler()
// {
// }



class linear_trans
{
    private:
        double _RAD1=180/3.14159;
        int _add_count=0;
        double _data_last,_set_last;
        int _add_count_arr[3]={0};
        double _last_arr[3]={0},_set_arr_last[3]={0};
        int cycle_arr[3]={0};
        bool _conver_done_flag=true;
        double data[3], set[3];
    public:

        double linearConvert(double data,double set,int cycle)
        {   
            if(_set_last!=set)
            {
                _add_count=0;
                _set_last=set;
            }

            if(data!=set)
            {   
                if(_add_count==0)
                    _data_last=data;

                data=_data_last+(set-_data_last)*_add_count/cycle;
                _add_count++;
                _conver_done_flag=false;
                // printf("%d _data_last:%f data:%f set:%f \n",_add_count,_data_last,data,set);
                if(_add_count==cycle)
                {
                    data=_data_last=set;
                    _add_count=0;
                    _conver_done_flag=true;
                    // printf("%d _data_last:%f data:%f set:%f \n",_add_count,_data_last,data,set);
                    // printf("Done!\n");
                }
            }
            return data;
        }

        bool retConvDoneFlag(void)
        {
            return _conver_done_flag;
        }

        double * linearConvert(double * data,double * set,int cycle)
        {   
            for (int i = 0; i < 3; i++)
            {

                if(_set_arr_last[i]!=set[i])
                {
                    _add_count_arr[i]=0;
                    _set_arr_last[i]=set[i];
                }

                if(data[i]!=set[i])
                {
                    if(_add_count_arr[i]==0)
                        _last_arr[i]=data[i];

                    data[i]=_last_arr[i]+(set[i]-_last_arr[i])*_add_count_arr[i]/cycle;
                    _add_count_arr[i]++;
                    _conver_done_flag=false;
                    if(_add_count_arr[i]==cycle)
                    {
                        data[i]=_last_arr[i]=set[i];
                        _add_count_arr[i]=0;
                         _conver_done_flag=true;
                        // printf("%d _data_last:%f data:%f set:%f \n",_add_count_arr[i],_last_arr[i],data[i],set[i]);
                    }
                }
            }
            return data;
        }

        Eigen::Vector3d linearConvert(Eigen::Vector3d data_, Eigen::Vector3d set_,int cycle)
        {   
            for (int i = 0; i < 3; i++)
            {

                if(_set_arr_last[i]!=set_(i))
                {
                    _add_count_arr[i]=0;
                    _set_arr_last[i]=set_(i);
                }

                if(data_(i)!=set_(i))
                {
                    if(_add_count_arr[i]==0)
                        _last_arr[i]=data_(i);

                    data_(i)=_last_arr[i]+(set_(i)-_last_arr[i])*_add_count_arr[i]/cycle;
                    _add_count_arr[i]++;
                    _conver_done_flag=false;
                    if(_add_count_arr[i]==cycle)
                    {
                        data_(i)=_last_arr[i]=set_(i);
                        _add_count_arr[i]=0;
                         _conver_done_flag=true;
                        // printf("%d _data_last:%f data:%f set:%f \n",_add_count_arr[i],_last_arr[i],data_(i),set_(i));
                    }
                }
            }

            return data_;
        }

        // Eigen::Matrix<double, 1, 6> linearConvert_Matrix_1_6(Eigen::Matrix<double, 1, 6> data_, Eigen::Matrix<double, 1, 6> set_,int cycle)
        // {   
        //     for (int i = 0; i < 6; i++)
        //     {

        //         if(_set_arr_last[i]!=set_(i))
        //         {
        //             _add_count_arr[i]=0;
        //             _set_arr_last[i]=set_(i);
        //         }

        //         if(data_(i)!=set_(i))
        //         {
        //             if(_add_count_arr[i]==0)
        //                 _last_arr[i]=data_(i);

        //             data_(i)=_last_arr[i]+(set_(i)-_last_arr[i])*_add_count_arr[i]/cycle;
        //             _add_count_arr[i]++;
        //             _conver_done_flag=false;
        //             if(_add_count_arr[i]==cycle)
        //             {
        //                 data_(i)=_last_arr[i]=set_(i);
        //                 _add_count_arr[i]=0;
        //                  _conver_done_flag=true;
        //                 // printf("%d _data_last:%f data:%f set:%f \n",_add_count_arr[i],_last_arr[i],data_(i),set_(i));
        //             }
        //         }
        //     }

        //     return data_;
        // }

        /**
        * @brief 
        * @author lcc
        */
        double * linearConvertByDifferValue(double * data,double * set,int set_differ_v)
        {   
            for (int i = 0; i < 3; i++)
            {

                if(_set_arr_last[i]!=set[i])
                {
                    _add_count_arr[i]=0;
                    _set_arr_last[i]=set[i];

                    cycle_arr[i]=fabs( data[i]*_RAD1 -  set[i]*_RAD1  ) / set_differ_v;
                    if( cycle_arr[i]<=10)
                         cycle_arr[i]=10;
                }

                printf(" cycle_arr[%d]:%d \n",i,cycle_arr[i]);
                if(data[i]!=set[i])
                {



                    if(_add_count_arr[i]==0)
                        _last_arr[i]=data[i];

                    data[i]=_last_arr[i]+(set[i]-_last_arr[i])*_add_count_arr[i]/cycle_arr[i];
                    _add_count_arr[i]++;
                    _conver_done_flag=false;
                    if(_add_count_arr[i]==cycle_arr[i])
                    {
                        data[i]=_last_arr[i]=set[i];
                        _add_count_arr[i]=0;
                         _conver_done_flag=true;
                        // printf("%d _data_last:%f data:%f set:%f \n",_add_count_arr[i],_last_arr[i],data[i],set[i]);
                    }
                }
            }
            return data;
        }

};


double * FourthPlane(double CPGXout,int CPGYout);

class neural_bezier_curve
{
    private:
        double delaT=0.005;
        double N=1/delaT; //%  200段，201个点
        double t=0;
        double Postion[3]={0};
        //%%%%%%%%% X 方向
        int n=9;   //%% 控制点数 n  
        int layerNum=n-1;   //% n层神经网络能够控制n个位置点，n-1段曲线，所以有n-1个可控制参数

        double  PX[9]={0};
        double  PY[9]={0};
        
        int TnIndex=0;

        double WXout=0,wx=0,WYout=0,wy=0;

        MatrixXd XCOM_BM,YCOM_BM;
        // %%%%%%%%%% X 方向
        MatrixXd Xoutput,Xinput,Xoutput0,Youtput,Yinput,Youtput0;
        MatrixXd kX,kY,aX,aY,XCn,YCn;
        // MatrixXd lamdaX,lamdaY;
        // Eigen::Matrix<double,1,6> lamdaX,lamdaY;

        double Xinput_[8]={0},Xinput_last[8]={0};  // layerNum=n-1=8
        double Xoutput_[8]={0},Xoutput_last[8]={0};
        double Xoutput0_=0,Xoutput0_last[8]={0};
        double Yinput_[8]={0},Yinput_last[8]={0};
        double Youtput_[8]={0},Youtput_last[8]={0};
        double Youtput0_=0,Youtput0_last[8]={0};
        double XCOM_BM_=0,XCOM_BM_last[8]={0};
        double YCOM_BM_=0,YCOM_BM_last[8]={0};

        double comb(double n,double m);
    public:

        // double  Set_PX[9]={-3 , -2.5 , -1.5 , -0.5 , 0 , 0.5 , 1.5 , 2.5 , 3 };  //　之前一直用的轨迹
        // double  Set_PY[9]={0*2 , 1.5*2 , 2.5*2 , 3.5*2 , 4.5*2 , 3.5*2 , 2.5*2 , 1.5*2 , 0*2 };


        Matrix<double,1,9> Set_PX,Set_PY;
        // double  Set_PX[9]={-3 ,-2.5, -2.3, -0.5, 1, 2.7, 3, 4.0, 3};  //可调整的轨迹
        // double  Set_PY[9]={0 , 3 , 5 , 7 , 9 , 7 , 5 , 3 , 0 };


        // double  Set_PX[9]={-3.75 ,-2.5, -2.3, 0, 1.25, 2.7, 3.75, 5.0, 3.75};  //论文轨迹
        // double  Set_PY[9]={0,4,8,7.5,4,8,6,2,0};

        MatrixXd lamdaX,lamdaY;
        double height_k=1,length_k=1;
        double height_k_last=0,length_k_last=0;

        linear_trans lamdaX_conver[9],lamdaY_conver[9];

        Eigen::Vector3d  bezierCurve(double _Cpg,double phase);
        
        void setCurveHight(double h_k)
        {
            if(height_k_last!=height_k)
            {
                 
                for(int i=0;i<n;i++)
                {
                    PY[i]=Set_PY(i)*height_k;
                    // printf("PY:%f ",PY[i]);
                }
                height_k_last=height_k;  
                // printf("h_k:%f \n",height_k);
            }

        }
        void setCurveLength(double l_k)
        {
            if(length_k_last!=length_k)
            {
                for(int i=0;i<n;i++)
                    PX[i]=Set_PX(i)*length_k;   
                length_k_last=length_k;    
            }
        }
        
        neural_bezier_curve()
        {
            //！ 20231127 lcc轨迹
            Set_PX<< -3 ,-2.5, -2.3, -0.5, 1, 2.7, 3, 4.0, 3;  //输入单位cm
            Set_PY<< 0 , 3 , 5 , 7 , 9 , 7 , 5 , 3 , 0 ;

            //！ 20231127 ypf轨迹
            // Set_PX<< -3.75 ,-2.5, -2.3, 0, 1.25, 2.7, 3.75, 5, 3.75;  //输入单位cm
            // Set_PY<<     0 , 2 , 4 , 3.5 , 2 , 4 , 3 , 0 , 0 ;

            for(int i=0;i<n;i++) //cm转m
            {
                Set_PX(i)=Set_PX(i)*0.01;
                Set_PY(i)=Set_PY(i)*0.01;
            }

            kX.resize(1,n);
            kY.resize(1,n);
            aX.resize(1,n);
            aY.resize(1,n);
            XCn.resize(1,n);
            YCn.resize(1,n);

            Xinput.resize(layerNum,N+1);
            Xoutput.resize(layerNum,N+1);
            Xoutput0.resize(1,N+1);

            Yinput.resize(layerNum,N+1);
            Youtput.resize(layerNum,N+1);
            Youtput0.resize(1,N+1);
            
            XCOM_BM.resize(1,N+1);
            YCOM_BM.resize(1,N+1);

            lamdaX.resize(1,n);
            lamdaY.resize(1,n);

            lamdaX.setZero();
            lamdaY.setZero();

            //！ 20231127 ypf轨迹
            // lamdaX(1) = 30;
            // lamdaX(3) = 60;
            // lamdaX(5) = 60;
        }

        double retDesStepLength(void)
        {   
            double step_length;
            step_length=PX[n-1]-PX[0];
            return step_length;
        }

        double retDesStepHight(void)
        {   
            double step_hight;
            step_hight=PY[n-1]-PY[0];
            // printf("PY[n-1]:%f \n",PY[n-1]);
            return step_hight;
        }

};


#include <iostream>
#include <chrono>
 
using namespace std;
using namespace std::chrono;
 
class TimeMteter
{
    private:
        time_point<high_resolution_clock>_start;
        Eigen::Matrix<double,1,6> cpg_period_count;
        Eigen::Matrix<double,1,6> cpg_scheduler_last;
    public:
    TimeMteter()
    {
        update();
    }
    
    ~TimeMteter()
    {
        ;
    }
    
    void update()
    {
        _start = high_resolution_clock::now();
    }

    //获取秒
    double getTimerSecond()
    {
        return getTimerMicroSec() * 0.000001;
    }

    //获取毫秒
    double getTimerMilliSec()
    {
        return getTimerMicroSec()*0.001;
    }

    //获取微妙
    long long getTimerMicroSec()
    {
        //当前时钟减去开始时钟的count
        return duration_cast<microseconds>(high_resolution_clock::now() - _start).count();
    }

    Eigen::Matrix<double,1,6> retSchedulerCount( Eigen::Matrix<int,1,6> cpg_scheduler )
    {
        for(int i=0;i<6;i++)
        {
            if( cpg_scheduler(i)==0 && cpg_scheduler_last(i)==1 )
            {
                cpg_period_count(i)++;
            }
            cpg_scheduler_last(i) = cpg_scheduler(i);
        }
        // std::cout<<cpg_period_count<<std::endl;
        return cpg_period_count;
    }
};


// class YQRDL
// {
//     public:

// };



#endif
