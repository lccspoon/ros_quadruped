 
// #ifdef COMPILE_WITH_ROS

#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include <string>
#include "common/unitreeRobot.h"

class IOSDK : public IOInterface{
public:
IOSDK();
~IOSDK();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

Eigen::Matrix<double, 3, 1> retSimOdeBodyP();

Eigen::Matrix<double, 3, 1> retSimOdeBodyV();

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);


};

#endif  // IOROS_H

// #endif  // COMPILE_WITH_ROS