#ifndef _INTL_SPI_H
#define _INTL_SPI_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <time.h>
#include <iostream>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#define datesize 47
class spi_sr
{

private:
    // spi_sr();
    // ~spi_sr();

    float _rad=180/3.1415926;
    int fd_spi0=-1;
    int fd_spi1=-1;
    uint32_t spiMode = SPI_MODE_0; // 0  0
    uint8_t bitsPerWord = 16;//8

    uint32_t speed =8000000; // 10500000Hz
        // uint32_t speed =20000000; // 10500000Hz

    const char *device0 = "/dev/spidev1.0";  //spi1.0 spi1.1
    const char *device1 = "/dev/spidev1.1";  //spi1.0 spi1.1
    uint16_t delay=0;
    size_t word_len = 2; // 16 bit word
    uint16_t txBufferf[datesize] = {0};
    uint16_t txBufferff[datesize] = {0};
    uint16_t rxBufferf[datesize];    
    uint16_t rxBufferff[datesize]; 

	uint16_t motor_disable0[46]={0x2255,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD};
	uint16_t motor_disable1[46]={0x2244,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD,
							0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFD};
	uint16_t motor_enable0[46]={0x2255,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC};
	uint16_t motor_enable1[46]={0x2244,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC,
								0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFC};

    int stop=2;

    //初始化
    int init_spi(void);

    //数据转化
    float spiuint_to_float(int x_int, float x_min, float x_max, int bits);
    int spifloat_to_uint(float x, float x_min, float x_max, int bits);
    void datetofloat(uint16_t *Rxdate);
    void datetofloat2(uint16_t *Rxdate); 

    //校验
    uint16_t calculateXORChecksum(const uint16_t* data, size_t len);

    //数据发送与接收
    int send_date(uint16_t *Txdate);
    int send_date2(uint16_t *Txdate);

    int send_recv(uint16_t *Txdate0, uint16_t *Txdate1);
public:

    struct MotorCmd{
        float q;
        float dq;
        float tau;
        float Kp;
        float Kd;
        MotorCmd(){
            q = 0;
            dq = 0;
            tau = 0;
            Kp = 0;
            Kd = 0;
        }
    } motor_cmd[18];

    struct MotorStates{
        float q;
        float dq;
        float tau;
        MotorStates(){
            q = 0;
            dq = 0;
            tau = 0;
        }
    } motor_states[18];

    spi_sr();
    ~spi_sr();

    // void enter_close_loop();
    void exit_close_loop();

    //数据打包+发送与接收
    void pos_loaddate2(float  pos0,float   pos1,float   pos2,float  pos3,float   pos4,float   pos5,float   pos6,float   pos7,float   pos8);
    void pos_loaddate(float  pos0,float   pos1,float   pos2,float  pos3,float   pos4,float   pos5,float   pos6,float   pos7,float   pos8);
};

# endif