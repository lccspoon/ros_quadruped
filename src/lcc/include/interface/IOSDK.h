 
// #ifdef COMPILE_WITH_ROS

#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include"interface/intl_spi.h"
#include <string>
#include "common/unitreeRobot.h"

#if USE_A_REAL_HEXAPOD == true
extern spi_sr spi_2;
extern bool MOTOR_DISABEL_FLAG;
extern bool MOTOR_ENABLE_FLAG;
extern bool MOTOR_READY_FLAG;
extern bool MOTOR_DATA_LOAD;
extern bool MOTOR_ENTER_CLOSELOOP;
extern float DOU_DONG_ANGEL;
extern bool TEST_FLAG;
extern Vec36 rec_offset;
#endif

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