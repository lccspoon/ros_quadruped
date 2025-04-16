#include "interface/CmdPanel.h"
#include "interface/KeyBoard.h"

UserValue_lcc userValue_lcc;

UserValue_lcc::UserValue_lcc(/* args */)
{
    lx = 0;
    ly = 0;
    rx = 0;
    ry = 0;
    L2 = 0;
}

UserValue_lcc::~UserValue_lcc()
{
}

void UserValue_lcc::setZero(){
    L2 = lt_l2.linearConvert(L2, 0, 200);
    lx = lt_lx.linearConvert(lx, 0, 200);
    ly = lt_ly.linearConvert(ly, 0, 200);
    rx = lt_rx.linearConvert(rx, 0, 200);
    ry = lt_ry.linearConvert(ry, 0, 200);

    // lx = lx -0.001;
    // printf(" lx:%f, ly:%f, rx:%f, ry:%f, L2:%f\n", lx,ly,rx,ry,L2);
    if( lt_l2.retConvDoneFlag() == true && 
        lt_lx.retConvDoneFlag() == true && 
        lt_ly.retConvDoneFlag() == true && 
        lt_rx.retConvDoneFlag() == true && 
        lt_ry.retConvDoneFlag() == true)
    {
        USVLCC_SETZERO = false;   
    }
}