#ifndef __IMU_H
#define __IMU_H

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "interface/serial_port.h"
#include "interface/hipnuc.h"


extern hipnuc_raw_t hipnuc_raw;
int imu_run(void);

#endif 