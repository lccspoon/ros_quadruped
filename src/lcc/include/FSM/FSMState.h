 
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"

// 基基类
class FSMState{
public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;}

    FSMStateName _stateName;
    std::string _stateNameString;
    // UserFunctionMode *_userFunctionMode; // lcc 20250601
    LowlevelState *_lowState;  // struct，包含了： UserCommand userCmd， UserValue userValue;

protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;

    LowlevelCmd *_lowCmd;

    // LowlevelState *_lowState;  // struct，包含了： UserCommand userCmd， UserValue userValue;
    // UserValue _userValue;  //lcc: 在每个状态里，都会有  _userValue = _lowState->userValue; 感觉有点多此一举，其实直接调用 _lowState->userValue 就完事了。
};

#endif  // FSMSTATE_H