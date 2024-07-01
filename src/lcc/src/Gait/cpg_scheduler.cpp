#include "Gait/cpg_scheduler.h"
double CPG::f(double x,double g,double K,int i )
{
    f_ret=alpha*(u-pow(x,2)-pow(g,2))*(x- u1)-(M_PI/(beta(i)*(exp(- a*g)+1)* T)
        +M_PI/((1-beta(i))*(exp( a*g)+1)* T))*(g- u2)+ K * (RR(0,i) -  nodeNum * x);//-sign(y)*F(i)+FL;

    return  f_ret;
}

double CPG::g(double x,double g,double K,int i )
{
    g_ret= alpha*( u-pow(x,2)-pow(g,2))*(g- u2)+(M_PI/(beta(i)*(exp(- a*g)+1)* T)
        +M_PI/((1-beta(i))*(exp( a*g)+1)* T))*(x- u1)+ K * (RR(1,i)-  nodeNum * g); 
    
    return  g_ret;
}

MatrixXd CPG::synFunc(Matrix<double,1,6>fi,Matrix<double,1,6>beta) 
{
    if(cpg_stop_flag==0)
    {
        for(int Ri=0;Ri< nodeNum;Ri++)
        {
            Rvalue.setZero();
            for(int Rj=0;Rj< nodeNum;Rj++)
            {
                RDelta(Ri,Rj)=(fi(Ri)-fi(Rj))*2*M_PI;
                RR1(Ri,Rj)=cos(RDelta(Ri,Rj));
                RR2(Ri,Rj)=-sin(RDelta(Ri,Rj));
                RR3(Ri,Rj)=sin(RDelta(Ri,Rj));
                RR4(Ri,Rj)=cos(RDelta(Ri,Rj));
                Rvalue22<<  RR1(Ri,Rj),RR2(Ri,Rj), RR3(Ri,Rj),RR4(Ri,Rj);
                Rvalue21=XX_YY.block(0, Rj, 2, 1);
                Rvalue=Rvalue+Rvalue22*Rvalue21;
            }
            RRnode(0,Ri)=Rvalue(0);
            RRnode(1,Ri)=Rvalue(1);
        }
        RR = RRnode;

        for(int i=0;i< legNum;i++)
        {
            if( ( t> Changtime) && ( changestate==0)  ) //change 结束后执行K值衰减
            {
                    K= iniK*exp(-5*( t- Changtime));
                if( K<0.001)  K=0;
            }

            //%龙格库塔法求下一点X,Y信号值

            k1(i)=f(X(i),Y(i), K,i);
            l1(i)=g(X(i),Y(i), K,i);
            k2(i)=f(X(i)+( control_cycle/2)*k1(i),Y(i)+( control_cycle/2)*l1(i), K,i);
            l2(i)=g(X(i)+( control_cycle/2)*k1(i),Y(i)+( control_cycle/2)*l1(i), K,i);
            k3(i)=f(X(i)+( control_cycle/2)*k2(i),Y(i)+( control_cycle/2)*l2(i), K,i);
            l3(i)=g(X(i)+( control_cycle/2)*k2(i),Y(i)+( control_cycle/2)*l2(i), K,i);
            k4(i)=f(X(i)+ control_cycle*k3(i),Y(i)+ control_cycle*l3(i), K,i);
            l4(i)=g(X(i)+ control_cycle*k3(i),Y(i)+ control_cycle*l3(i), K,i);
            X(i)=X(i)+( control_cycle/6)*(k1(i)+2*k2(i)+2*k3(i)+k4(i));
            Y(i)=Y(i)+( control_cycle/6)*(l1(i)+2*l2(i)+2*l3(i)+l4(i));
        }
    }

    output<< X,Y;

    // std::cout<<"X"<<std::endl;
    // std::cout<<X<<std::endl;
    // std::cout<<"cpg_stop_flag"<<std::endl;
    // std::cout<<cpg_stop_flag<<std::endl;

    return output;
}

void CPG::cpgNewParaInit(void)
{
    for(int index=0;index<6;index++)
    {
        X(index)=1-0.02*index;
        Y(index)=0;
    }
        iniK=5; K= iniK;
    savedbeta.setZero();
    beta0<< 1,1,1,1,1,1;
    fi<< 0,0+(1- gait),0+2*(1- gait),0+3*(1- gait),0+4*(1- gait),0+5*(1- gait);  //通过相位差确定fi的值。
    // std::cout<<X<<std::endl;
    // std::cout<<Y<<std::endl;
    
}

int NanFlag=0;
Eigen::Matrix<double,2,6> cpg_scheduler;
Eigen::Matrix<double,1,6> cpg_touch_dow_scheduler;
Eigen::Matrix<double,2,6> CPG::cpgNewRun(void)
{
    // printf("cpg  n:%d\n",n);
    do
    {   
        if(NanFlag==0)
        {
            cpgNewParaInit();
            NanFlag=1;
        }

        beta= betatemp*beta0;
        fi<< 0,0+(1- gait),0+2*(1- gait),0+3*(1- gait),0+4*(1- gait),0+5*(1- gait);  //通过相位差确定fi的值。

        XX_YY=synFunc(fi,beta);
        XX=XX_YY.block(0,0,1,6);
        YY=XX_YY.block(1,0,1,6);
            // k=n;
            t= k* control_cycle;

        for(int index=0;index<6;index++)
        {
            if(YY(index)<=0)  Phase[index]=1;
            else  Phase[index]=0;
            // printf("%d  XX(%d): %f YY: %f  Block:%d\n", k,XX(1),YY(1) , Phase[0]);
        }

        for(int i=0;i<6;i++)
            cpg_touch_dow_scheduler(i)=Phase[i];
            
        cpg_scheduler.block<1,6>(0,0)=XX;
        cpg_scheduler.block<1,6>(1,0)=cpg_touch_dow_scheduler;

    } while ( cpg_touch_dow_scheduler(0,0)>1 );

    return cpg_scheduler;
}
