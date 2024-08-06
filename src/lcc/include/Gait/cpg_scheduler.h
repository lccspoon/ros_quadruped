#ifndef _CPG_H
#define _CPG_H



// #include "../utilities/linear_trans.h"
#include <iostream>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace Eigen;

class CPG
{
    private:
        int legNum=6,nodeNum=6,k=0,n=0;
        int changestate=0,iniK=5;
        double t=0;
        double K=0;
        int u1=0,u2=0,alpha=100,a=50,u=1;
        double f_ret=0,g_ret=0;
        double betatemp=0.5;
        int Phase[6];
        int Changtime=1,savedgait=0,savedT=0;
        int CPGstart_flag=0;
        double CPGXout[6]={0},CPGYout[6]={0};
        Matrix<double,2,1> Rvalue;
        Matrix<double,6,6> RDelta;
        Matrix<double,2,6> RRnode;
        Matrix<double,2,6> RR;
        Matrix<double,2,6> output;
        Matrix<double,6,6> RR1;
        Matrix<double,6,6> RR2;
        Matrix<double,6,6> RR3;
        Matrix<double,6,6> RR4;
        Matrix<double,2,2> Rvalue22;
        Matrix<double,2,1> Rvalue21;
        Matrix<double,2,6> XX_YY;
        Matrix<double,1,6> k1,k2,k3,k4,l1,l2,l3,l4,X,Y,beta,savedbeta,beta0,fi,XX,YY;

        MatrixXd synFunc(Matrix<double,1,6>fi,Matrix<double,1,6>beta);
        void cpgNewParaInit(void);
        double f(double x,double g,double K,int i );
        double g(double x,double g,double K,int i );

    public:
        double gait=0.5;                      //lcc 20230409:   gait=0.5 默认为三角步态
        // double control_cycle=0.0005,T=1;       
        double control_cycle=0.00025,T=1;       
        // linear_trans GaitLinTran,CtrlCyclLinTran,TLinTran;
        Eigen::Matrix<double,2,6>  cpgNewRun(void);

        int cpg_stop_flag=0;

        CPG()
        {
            Rvalue.setZero();
            RDelta.setZero();
            RRnode.setZero();
            RR.setZero();
            output.setZero();
            RR1.setZero();
            RR2.setZero();
            RR3.setZero();
            RR4.setZero();
            Rvalue22.setZero();
            Rvalue21.setZero();
            XX_YY.setZero();
            k1.setZero();k2.setZero();k3.setZero();k4.setZero();l1.setZero();l2.setZero();
            l3.setZero();l4.setZero();X.setZero();Y.setZero();beta.setZero();savedbeta.setZero();
            beta0.setZero();fi.setZero();XX.setZero();YY.setZero();

            for(int index=0;index<6;index++)
            {
                X(index)=1-0.02*index;
                Y(index)=0;
            }
                iniK=5; K= iniK;
            savedbeta.setZero();
            beta0<< 1,1,1,1,1,1;
            fi<< 0,0+(1- gait),0+2*(1- gait),0+3*(1- gait),0+4*(1- gait),0+5*(1- gait);  //通过相位差确定fi的值。

        }



};



#endif