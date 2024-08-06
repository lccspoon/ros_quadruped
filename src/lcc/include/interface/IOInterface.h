 
#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

class IOInterface{
public:
IOInterface(){}
~IOInterface(){delete cmdPanel;}

//在函数声明的末尾加上 = 0，其含义是该函数在基类中没有实现，并且派生类（子类）必须提供该函数的具体实现。
//这种机制主要用于定义接口和抽象基类。
virtual void sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
void zeroCmdPanel(){cmdPanel->setZero();}
void setPassive(){cmdPanel->setPassive();}

protected:
CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H