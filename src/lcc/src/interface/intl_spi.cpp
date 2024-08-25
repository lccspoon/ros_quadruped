#include"interface/intl_spi.h"
#include "zlib.h"

spi_sr::spi_sr(){
	init_spi();
	usleep(5000);
	send_date(motor_disable0);///远离派镇
	usleep(100);
	send_date2(motor_disable1);//	//靠近电源侧
	usleep(2500);
	send_date(motor_disable0);//靠近电源侧S1	//靠近hzg附近//远离派镇
	usleep(100);
	send_date2(motor_disable1);//远离电源侧S2	//靠近电源侧
	usleep(2500);

    rec_moter_q.setZero();
    rec_moter_v.setZero();
    rec_moter_t.setZero();
    rec_moter_q_last.setZero();
    rec_moter_v_last.setZero();
    rec_moter_t_last.setZero();
    rec_moter_q_erroCount.setZero();
    rec_moter_v_erroCount.setZero();
    rec_moter_t_erroCount.setZero();
}

spi_sr::~spi_sr(){
}

float spi_sr::spiuint_to_float(int x_int, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int spi_sr::spifloat_to_uint(float x, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        float offset = x_min;

        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

int spi_sr::init_spi(){
    fd_spi0 = open(device0, O_RDWR);
    fd_spi1 = open(device1, O_RDWR);

    printf("fd_spi0:%d\n",fd_spi0);
    printf("fd_spi1:%d\n",fd_spi1);

    if (fd_spi0 < 0)
    {
        printf("fd_spi0:%d",fd_spi0);
        std::cerr << "spi opening SPI0 device." << std::endl;
        return 1;
    }
    if (fd_spi1 < 0)
    {
        printf("fd_spi0:%d",fd_spi1);
        std::cerr << "spi opening SPI1 device." << std::endl;
        return 1;
    }

     // 设置SPI模式
    if (ioctl(fd_spi0, SPI_IOC_WR_MODE32, &spiMode) == -1)
    {
        std::cerr << "Error setting SPI mode." << std::endl;
        return 1;
    }
    if (ioctl(fd_spi0, SPI_IOC_RD_MODE32, &spiMode) == -1)
    {
        std::cerr << "Error getting SPI mode." << std::endl;
        return 1;
    }
    if (ioctl(fd_spi1, SPI_IOC_WR_MODE32, &spiMode) == -1)
    {
        std::cerr << "Error setting SPI mode." << std::endl;
        return 1;
    }
    if (ioctl(fd_spi1, SPI_IOC_RD_MODE32, &spiMode) == -1)
    {
        std::cerr << "Error getting SPI mode." << std::endl;
        return 1;
    }

    // 设置SPI字长
    if (ioctl(fd_spi0, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) == -1)
    {
        std::cerr << "Error setting SPI bits per word." << std::endl;
        return 1;
    }
        if (ioctl(fd_spi0, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord) == -1)
    {
        std::cerr << "Error setting SPI bits per word." << std::endl;
        return 1;
    }
    if (ioctl(fd_spi1, SPI_IOC_WR_BITS_PER_WORD, &bitsPerWord) == -1)
    {
        std::cerr << "Error setting SPI bits per word." << std::endl;
        return 1;
    }
        if (ioctl(fd_spi1, SPI_IOC_RD_BITS_PER_WORD, &bitsPerWord) == -1)
    {
        std::cerr << "Error setting SPI bits per word." << std::endl;
        return 1;
    }

    // 设置SPI时钟速度
    if (ioctl(fd_spi0, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
    {
        std::cerr << "Error setting SPI speed." << std::endl;
        return 1;
    }
    if (ioctl(fd_spi0, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1)
    {
        std::cerr << "Error getting SPI speed." << std::endl;
        return 1;
    }
    if (ioctl(fd_spi1, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
    {
        std::cerr << "Error setting SPI speed." << std::endl;
        return 1;
    }
    if (ioctl(fd_spi1, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1)
    {
        std::cerr << "Error getting SPI speed." << std::endl;
        return 1;
    }
    return 0;
}

uint16_t spi_sr::calculateXORChecksum(const uint16_t* data, size_t len)
{
    const uint16_t checksum = 0x1D;
    uint16_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 16; ++j)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ checksum;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

int spi_sr::send_date(uint16_t *Txdate){
    memcpy(txBufferf,Txdate,46*sizeof(uint16_t));
    txBufferf[46] = calculateXORChecksum(Txdate, 46);

    struct spi_ioc_transfer trf0[1];
    memset(trf0, 0, 1 * sizeof(struct spi_ioc_transfer));
    trf0[0].bits_per_word = bitsPerWord;
    trf0[0].cs_change = 0; //cs_change 为0 片选会变化，为1就恒为低电平
    trf0[0].delay_usecs = 0;
    trf0[0].len = datesize * sizeof(uint16_t);
    trf0[0].rx_buf = (unsigned long)rxBufferf;
    trf0[0].tx_buf = (unsigned long)txBufferf;
    trf0[0].speed_hz = speed;
    // cs.lm_gpio(0);
    if (ioctl(fd_spi0, SPI_IOC_MESSAGE(1), &trf0) == -1)
    {
         std::cerr << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@qqqqqqError during SPI transfer." << std::endl;
        return 1;
    }
    // cs.lm_gpio(1);
    for (int i = 0; i < datesize; i++)
    // for (int i = 0; i < 20; i++)
    {
        //S1+123=标号6
        //S1+456=标号5
        ///S1+789=标号4
    //    std::cout << "S1Received data: "<< std::dec  << i <<" : "<< std::hex << rxBufferf[i] << std::endl;
    }  
        // printf("\n");     

    datetofloat(rxBufferf);         
    return 0;
}

int spi_sr::send_date2(uint16_t *Txdate){
    memcpy(txBufferff,Txdate,46*sizeof(uint16_t));
    txBufferff[46] = calculateXORChecksum(Txdate, 46);

    struct spi_ioc_transfer trf[1];
    memset(trf, 0, 1 * sizeof(struct spi_ioc_transfer));

    trf[0].bits_per_word = bitsPerWord;
    trf[0].cs_change = 0; //cs_change 为0 片选会变化，为1就恒为低电平
    trf[0].delay_usecs = 0;
    trf[0].len = datesize * sizeof(uint16_t);
    trf[0].rx_buf = (unsigned long)rxBufferff;
    trf[0].tx_buf = (unsigned long)txBufferff;
    trf[0].speed_hz = speed;
    if (ioctl(fd_spi1, SPI_IOC_MESSAGE(1), &trf) == -1)
    {
        std::cerr << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@Error during SPI transfer." << std::endl;
        return 1;
    }
    for (int i = 0; i < datesize; i++)
    {
        // printf("i:%d\n",i);
        //S2+456=标号3  
        //S2+123=标号2
        //S2+789=标号1
    //    std::cout << "S2Received data: "<< std::dec << (int)i <<" : "<< std::hex << (int)rxBufferff[i] << std::endl;  
    }   
    // printf("\n");     
    datetofloat2(rxBufferff);   
    return 0;
}
int numid0=0,numid1=0,numid2=0,numid3=0,numid4=0,numid5=0,numid6=0,numid7=0,numid8=0;
int nnumid0=0,nnumid1=0,nnumid2=0,nnumid3=0,nnumid4=0,nnumid5=0,nnumid6=0,nnumid7=0,nnumid8=0;

void spi_sr::datetofloat2(uint16_t *Rxdate)
{


    float spi_pos0 = 0;
    float spi_vel0 = 0;
    float spi_toq0 = 0;	
    uint16_t idd0=( Rxdate[2]<<4);
    if(idd0!=16)
    {
        idd0=(Rxdate[3]<<4);
        if(idd0==16)
        {
             spi_pos0 = spiuint_to_float(Rxdate[4], -12.5, 12.5, 16);
             spi_vel0 = spiuint_to_float(Rxdate[5], -30.0f,30.0f, 12);
             spi_toq0 =  spiuint_to_float(Rxdate[6], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd0=0;
            ++numid0;
        }
    }
    else
    {
        spi_pos0 = spiuint_to_float(Rxdate[3], -12.5, 12.5, 16);
        spi_vel0 = spiuint_to_float(Rxdate[4], -30.0f,30.0f, 12);
        spi_toq0 =  spiuint_to_float(Rxdate[5], -10.0f, 10.0f, 12);	
    }

    float spi_pos1 = 0;
    float spi_vel1 = 0;
    float spi_toq1 = 0;	
    uint16_t idd1=( Rxdate[6]<<4);
    if(idd1!=32)
    {
        idd1=( Rxdate[7]<<4);
        if(idd1==32)
        {
             spi_pos1 = spiuint_to_float(Rxdate[8], -12.5, 12.5, 16);
             spi_vel1 = spiuint_to_float(Rxdate[9], -30.0f,30.0f, 12);
             spi_toq1 =  spiuint_to_float(Rxdate[10], -10.0f, 10.0f, 12);	            
        }
        else
        { 
            idd1 = 0;
            ++numid1;
        }
    }
    else
    {
         spi_pos1 = spiuint_to_float(Rxdate[7], -12.5, 12.5, 16);
         spi_vel1 = spiuint_to_float(Rxdate[8], -30.0f,30.0f, 12);
         spi_toq1 =  spiuint_to_float(Rxdate[9], -10.0f, 10.0f, 12);	
    }

    float spi_pos2 = 0;
    float spi_vel2 = 0;
    float spi_toq2 = 0;	
    uint16_t idd2=( Rxdate[10]<<4);
    if(idd2!=48)
    {
        idd2=( Rxdate[11]<<4);
        if(idd2==48)
        {
             spi_pos2 = spiuint_to_float(Rxdate[12], -12.5, 12.5, 16);
             spi_vel2 = spiuint_to_float(Rxdate[13], -30.0f,30.0f, 12);
             spi_toq2 =  spiuint_to_float(Rxdate[14], -10.0f, 10.0f, 12);	            
        }
        else
        {        
            idd2 = 0;
            ++numid2; 
        }
    }
    else
    {
             spi_pos2 = spiuint_to_float(Rxdate[11], -12.5, 12.5, 16);
             spi_vel2 = spiuint_to_float(Rxdate[12], -30.0f,30.0f, 12);
             spi_toq2 =  spiuint_to_float(Rxdate[13], -10.0f, 10.0f, 12);	   
    }
          
    float spi_pos3 = 0;
    float spi_vel3 = 0;
    float spi_toq3 = 0;	
    uint16_t idd3=( Rxdate[14]<<4);
    if(idd3!=16)
    {
        idd3=( Rxdate[15]<<4);
        if(idd3==16)
        {
             spi_pos3 = spiuint_to_float(Rxdate[16], -12.5, 12.5, 16);
             spi_vel3 = spiuint_to_float(Rxdate[17], -30.0f,30.0f, 12);
             spi_toq3 =  spiuint_to_float(Rxdate[18], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd3=0;
            ++numid3;
        }

    }
    else
    {
         spi_pos3 = spiuint_to_float(Rxdate[15], -12.5, 12.5, 16);
         spi_vel3 = spiuint_to_float(Rxdate[16], -30.0f,30.0f, 12);
         spi_toq3 =  spiuint_to_float(Rxdate[17], -10.0f, 10.0f, 12);	
    }

    float spi_pos4 = 0;
    float spi_vel4 = 0;
    float spi_toq4 = 0;	
    uint16_t idd4=( Rxdate[18]<<4);
    if(idd4!=32)
    {
        idd4=( Rxdate[19]<<4);
        if(idd4==32)
        {
             spi_pos4 = spiuint_to_float(Rxdate[20], -12.5, 12.5, 16);
             spi_vel4 = spiuint_to_float(Rxdate[21], -30.0f,30.0f, 12);
             spi_toq4 =  spiuint_to_float(Rxdate[22], -10.0f, 10.0f, 12);	 
        }
        else
        {
            idd4 = 0;             
            ++numid4;
        }
    }
    else
    {
         spi_pos4 = spiuint_to_float(Rxdate[19], -12.5, 12.5, 16);
         spi_vel4 = spiuint_to_float(Rxdate[20], -30.0f,30.0f, 12);
         spi_toq4 = spiuint_to_float(Rxdate[21], -10.0f, 10.0f, 12);	        
    }

    float spi_pos5 = 0;
    float spi_vel5 = 0;
    float spi_toq5 = 0;
    uint16_t idd5=( Rxdate[22]<<4);
    if(idd5!=48)
    {
        idd5=( Rxdate[23]<<4);
        if(idd5==48)
        {
             spi_pos5 = spiuint_to_float(Rxdate[24], -12.5, 12.5, 16);
             spi_vel5 = spiuint_to_float(Rxdate[25], -30.0f,30.0f, 12);
             spi_toq5 =  spiuint_to_float(Rxdate[26], -10.0f, 10.0f, 12);	        
        }
        else
        {
            idd5=0;
            ++numid5;	
        }
    }
    else
    {
         spi_pos5 = spiuint_to_float(Rxdate[23], -12.5, 12.5, 16);
         spi_vel5 = spiuint_to_float(Rxdate[24], -30.0f,30.0f, 12);
         spi_toq5 =  spiuint_to_float(Rxdate[25], -10.0f, 10.0f, 12);	   
    }

    float spi_pos6 = 0;
    float spi_vel6 = 0;
    float spi_toq6 = 0;	
    uint16_t idd6=( Rxdate[26]<<4);
    if(idd6!=16)
    {
        idd6=( Rxdate[27]<<4);
        if(idd6==16)
        {
             spi_pos6 = spiuint_to_float(Rxdate[28], -12.5, 12.5, 16);
             spi_vel6 = spiuint_to_float(Rxdate[29], -30.0f,30.0f, 12);
             spi_toq6 =  spiuint_to_float(Rxdate[30], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd6 = 0;
            ++numid6;
        }
    }
    else
    {
         spi_pos6 = spiuint_to_float(Rxdate[27], -12.5, 12.5, 16);
         spi_vel6 = spiuint_to_float(Rxdate[28], -30.0f,30.0f, 12);
         spi_toq6 =  spiuint_to_float(Rxdate[29], -10.0f, 10.0f, 12);	        
    }

    float spi_pos7 = 0;
    float spi_vel7 = 0;
    float spi_toq7 = 0;	
    uint16_t idd7=( Rxdate[30]<<4);
    if(idd7!=32)
    {
        idd7=( Rxdate[31]<<4);
        if(idd7==32)
        {
             spi_pos7 = spiuint_to_float(Rxdate[32], -12.5, 12.5, 16);
             spi_vel7 = spiuint_to_float(Rxdate[33], -30.0f,30.0f, 12);
             spi_toq7 =  spiuint_to_float(Rxdate[34], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd7=0;
            ++numid7;
        }
    }
    else
    {
         spi_pos7 = spiuint_to_float(Rxdate[31], -12.5, 12.5, 16);
         spi_vel7 = spiuint_to_float(Rxdate[32], -30.0f,30.0f, 12);
         spi_toq7 = spiuint_to_float(Rxdate[33], -10.0f, 10.0f, 12);	
    }

    float spi_pos8 = 0;
    float spi_vel8 = 0;
    float spi_toq8 = 0;
    uint16_t idd8=( Rxdate[34]<<4);
    if(idd8!=48)
    {
        idd8=( Rxdate[35]<<4);
        if(idd8==48)
        {
             spi_pos8 = spiuint_to_float(Rxdate[36], -12.5, 12.5, 16);
             spi_vel8 = spiuint_to_float(Rxdate[37], -30.0f,30.0f, 12);
             spi_toq8 = spiuint_to_float(Rxdate[38], -10.0f, 10.0f, 12);
        }
        else
        {
            idd8=0;
            ++numid8;
        }
    }
    else
    {
         spi_pos8 = spiuint_to_float(Rxdate[35], -12.5, 12.5, 16);
         spi_vel8 = spiuint_to_float(Rxdate[36], -30.0f,30.0f, 12);
         spi_toq8 = spiuint_to_float(Rxdate[37], -10.0f, 10.0f, 12);	        
    }
        //S2+456=标号3  
        //S2+123=标号2
        //S2+789=标号1

    //lm leg_3
    motor_states[9].q = spi_pos0; 
    motor_states[9].dq = spi_vel0; 
    motor_states[9].tau = spi_toq0;

    motor_states[10].q = spi_pos1; 
    motor_states[10].dq = spi_vel1; 
    motor_states[10].tau = spi_toq1;

    motor_states[11].q = spi_pos2; 
    motor_states[11].dq = spi_vel2; 
    motor_states[11].tau = spi_toq2;

    //rm leg_2
    motor_states[6].q = spi_pos3; 
    motor_states[6].dq = spi_vel3; 
    motor_states[6].tau = spi_toq3;

    motor_states[7].q = spi_pos4; 
    motor_states[7].dq = spi_vel4; 
    motor_states[7].tau = spi_toq4;

    motor_states[8].q = spi_pos5; 
    motor_states[8].dq = spi_vel5; 
    motor_states[8].tau = spi_toq5;

    //lf leg_1
    motor_states[3].q = spi_pos6; 
    motor_states[3].dq = spi_vel6; 
    motor_states[3].tau = spi_toq6;

    motor_states[4].q = spi_pos7; 
    motor_states[4].dq = spi_vel7; 
    motor_states[4].tau = spi_toq7;

    motor_states[5].q = spi_pos8; 
    motor_states[5].dq = spi_vel8; 
    motor_states[5].tau = spi_toq8;

    // std::cout << "lm1: "<<" id: "<<std::hex << idd0 <<" pos: "<<std::fixed<< spi_pos0*_rad<<" vel: "<<std::fixed<<spi_vel0<<" toq: "<<std::fixed<<spi_toq0<<std::endl;
    // std::cout << "lm2: "<<" id: "<<std::hex << idd1 <<" pos: "<<std::fixed<< spi_pos1*_rad<<" vel: "<<std::fixed<<spi_vel1<<" toq: "<<std::fixed<<spi_toq1<<std::endl;
    // std::cout << "lm3: "<<" id: "<<std::hex << idd2 <<" pos: "<<std::fixed<< spi_pos2*_rad<<" vel: "<<std::fixed<<spi_vel2<<" toq: "<<std::fixed<<spi_toq2<<std::endl;
    // std::cout << "rm1: "<<" id: "<<std::hex << idd3 <<" pos: "<<std::fixed<< spi_pos3*_rad<<" vel: "<<std::fixed<<spi_vel3<<" toq: "<<std::fixed<<spi_toq3<<std::endl;
    // std::cout << "rm2: "<<" id: "<<std::hex << idd4 <<" pos: "<<std::fixed<< spi_pos4*_rad<<" vel: "<<std::fixed<<spi_vel4<<" toq: "<<std::fixed<<spi_toq4<<std::endl;
    // std::cout << "rm3: "<<" id: "<<std::hex << idd5 <<" pos: "<<std::fixed<< spi_pos5*_rad<<" vel: "<<std::fixed<<spi_vel5<<" toq: "<<std::fixed<<spi_toq5<<std::endl;
    // std::cout << "lf1: "<<" id: "<<std::hex << idd6 <<" pos: "<<std::fixed<< spi_pos6*_rad<<" vel: "<<std::fixed<<spi_vel6<<" toq: "<<std::fixed<<spi_toq6<<std::endl;
    // std::cout << "lf2: "<<" id: "<<std::hex << idd7 <<" pos: "<<std::fixed<< spi_pos7*_rad<<" vel: "<<std::fixed<<spi_vel7<<" toq: "<<std::fixed<<spi_toq7<<std::endl;
    // std::cout << "lf3: "<<" id: "<<std::hex << idd8 <<" pos: "<<std::fixed<< spi_pos8*_rad<<" vel: "<<std::fixed<<spi_vel8<<" toq: "<<std::fixed<<spi_toq8<<std::endl;
    // std::cout << "error: "<<" 1: "<<std::dec << numid0 <<" 2: "<<std::dec<< numid1<<" 3: "<<std::dec<<numid2
    //             <<" 4: "<<std::dec << numid3 <<" 5: "<<std::dec<< numid4<<" 6: "<<std::dec<<numid5
    //             <<" 7: "<<std::dec << numid6 <<" 8: "<<std::dec<< numid7<<" 9: "<<std::dec<<numid8
    //             <<std::endl;

    // printf("\n");       
}


void spi_sr::datetofloat(uint16_t *Rxdate)
{
    float spi_pos0 = 0;
    float spi_vel0 = 0;
    float spi_toq0 = 0;	
    uint16_t idd0=( Rxdate[2]<<4);
    if(idd0!=16)
    {
        idd0=(Rxdate[3]<<4);
        if(idd0==16)
        {
             spi_pos0 = spiuint_to_float(Rxdate[4], -12.5, 12.5, 16);
             spi_vel0 = spiuint_to_float(Rxdate[5], -30.0f,30.0f, 12);
             spi_toq0 =  spiuint_to_float(Rxdate[6], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd0=0;
            ++numid0;
        }
    }
    else
    {
        spi_pos0 = spiuint_to_float(Rxdate[3], -12.5, 12.5, 16);
        spi_vel0 = spiuint_to_float(Rxdate[4], -30.0f,30.0f, 12);
        spi_toq0 =  spiuint_to_float(Rxdate[5], -10.0f, 10.0f, 12);	
    }

    float spi_pos1 = 0;
    float spi_vel1 = 0;
    float spi_toq1 = 0;	
    uint16_t idd1=( Rxdate[6]<<4);
    if(idd1!=32)
    {
        idd1=( Rxdate[7]<<4);
        if(idd1==32)
        {
             spi_pos1 = spiuint_to_float(Rxdate[8], -12.5, 12.5, 16);
             spi_vel1 = spiuint_to_float(Rxdate[9], -30.0f,30.0f, 12);
             spi_toq1 =  spiuint_to_float(Rxdate[10], -10.0f, 10.0f, 12);	            
        }
        else
        { 
            idd1 = 0;
            ++nnumid1;
        }
    }
    else
    {
         spi_pos1 = spiuint_to_float(Rxdate[7], -12.5, 12.5, 16);
         spi_vel1 = spiuint_to_float(Rxdate[8], -30.0f,30.0f, 12);
         spi_toq1 =  spiuint_to_float(Rxdate[9], -10.0f, 10.0f, 12);	
    }

    float spi_pos2 = 0;
    float spi_vel2 = 0;
    float spi_toq2 = 0;	
    uint16_t idd2=( Rxdate[10]<<4);
    if(idd2!=48)
    {
        idd2=( Rxdate[11]<<4);
        if(idd2==48)
        {
             spi_pos2 = spiuint_to_float(Rxdate[12], -12.5, 12.5, 16);
             spi_vel2 = spiuint_to_float(Rxdate[13], -30.0f,30.0f, 12);
             spi_toq2 =  spiuint_to_float(Rxdate[14], -10.0f, 10.0f, 12);	            
        }
        else
        {        
            idd2 = 0;
            ++nnumid2; 
        }
    }
    else
    {
             spi_pos2 = spiuint_to_float(Rxdate[11], -12.5, 12.5, 16);
             spi_vel2 = spiuint_to_float(Rxdate[12], -30.0f,30.0f, 12);
             spi_toq2 =  spiuint_to_float(Rxdate[13], -10.0f, 10.0f, 12);	   
    }
          
    float spi_pos3 = 0;
    float spi_vel3 = 0;
    float spi_toq3 = 0;	
    uint16_t idd3=( Rxdate[14]<<4);
    if(idd3!=16)
    {
        idd3=( Rxdate[15]<<4);
        if(idd3==16)
        {
             spi_pos3 = spiuint_to_float(Rxdate[16], -12.5, 12.5, 16);
             spi_vel3 = spiuint_to_float(Rxdate[17], -30.0f,30.0f, 12);
             spi_toq3 =  spiuint_to_float(Rxdate[18], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd3=0;
            ++nnumid3;
        }

    }
    else
    {
         spi_pos3 = spiuint_to_float(Rxdate[15], -12.5, 12.5, 16);
         spi_vel3 = spiuint_to_float(Rxdate[16], -30.0f,30.0f, 12);
         spi_toq3 =  spiuint_to_float(Rxdate[17], -10.0f, 10.0f, 12);	
    }

    float spi_pos4 = 0;
    float spi_vel4 = 0;
    float spi_toq4 = 0;	
    uint16_t idd4=( Rxdate[18]<<4);
    if(idd4!=32)
    {
        idd4=( Rxdate[19]<<4);
        if(idd4==32)
        {
             spi_pos4 = spiuint_to_float(Rxdate[20], -12.5, 12.5, 16);
             spi_vel4 = spiuint_to_float(Rxdate[21], -30.0f,30.0f, 12);
             spi_toq4 =  spiuint_to_float(Rxdate[22], -10.0f, 10.0f, 12);	 
        }
        else
        {
            idd4 = 0;             
            ++nnumid4;
        }
    }
    else
    {
         spi_pos4 = spiuint_to_float(Rxdate[19], -12.5, 12.5, 16);
         spi_vel4 = spiuint_to_float(Rxdate[20], -30.0f,30.0f, 12);
         spi_toq4 = spiuint_to_float(Rxdate[21], -10.0f, 10.0f, 12);	        
    }

    float spi_pos5 = 0;
    float spi_vel5 = 0;
    float spi_toq5 = 0;
    uint16_t idd5=( Rxdate[22]<<4);
    if(idd5!=48)
    {
        idd5=( Rxdate[23]<<4);
        if(idd5==48)
        {
             spi_pos5 = spiuint_to_float(Rxdate[24], -12.5, 12.5, 16);
             spi_vel5 = spiuint_to_float(Rxdate[25], -30.0f,30.0f, 12);
             spi_toq5 =  spiuint_to_float(Rxdate[26], -10.0f, 10.0f, 12);	        
        }
        else
        {
            idd5=0;
            ++nnumid5;	
        }
    }
    else
    {
         spi_pos5 = spiuint_to_float(Rxdate[23], -12.5, 12.5, 16);
         spi_vel5 = spiuint_to_float(Rxdate[24], -30.0f,30.0f, 12);
         spi_toq5 =  spiuint_to_float(Rxdate[25], -10.0f, 10.0f, 12);	   
    }

    float spi_pos6 = 0;
    float spi_vel6 = 0;
    float spi_toq6 = 0;	
    uint16_t idd6=( Rxdate[26]<<4);
    if(idd6!=16)
    {
        idd6=( Rxdate[27]<<4);
        if(idd6==16)
        {
             spi_pos6 = spiuint_to_float(Rxdate[28], -12.5, 12.5, 16);
             spi_vel6 = spiuint_to_float(Rxdate[29], -30.0f,30.0f, 12);
             spi_toq6 =  spiuint_to_float(Rxdate[30], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd6 = 0;
            ++nnumid6;
        }
    }
    else
    {
         spi_pos6 = spiuint_to_float(Rxdate[27], -12.5, 12.5, 16);
         spi_vel6 = spiuint_to_float(Rxdate[28], -30.0f,30.0f, 12);
         spi_toq6 =  spiuint_to_float(Rxdate[29], -10.0f, 10.0f, 12);	        
    }

    float spi_pos7 = 0;
    float spi_vel7 = 0;
    float spi_toq7 = 0;	
    uint16_t idd7=( Rxdate[30]<<4);
    if(idd7!=32)
    {
        idd7=( Rxdate[31]<<4);
        if(idd7==32)
        {
             spi_pos7 = spiuint_to_float(Rxdate[32], -12.5, 12.5, 16);
             spi_vel7 = spiuint_to_float(Rxdate[33], -30.0f,30.0f, 12);
             spi_toq7 =  spiuint_to_float(Rxdate[34], -10.0f, 10.0f, 12);	
        }
        else
        {
            idd7=0;
            ++nnumid7;
        }
    }
    else
    {
         spi_pos7 = spiuint_to_float(Rxdate[31], -12.5, 12.5, 16);
         spi_vel7 = spiuint_to_float(Rxdate[32], -30.0f,30.0f, 12);
         spi_toq7 = spiuint_to_float(Rxdate[33], -10.0f, 10.0f, 12);	
    }

    float spi_pos8 = 0;
    float spi_vel8 = 0;
    float spi_toq8 = 0;
    uint16_t idd8=( Rxdate[34]<<4);
    if(idd8!=48)
    {
        idd8=( Rxdate[35]<<4);
        if(idd8==48)
        {
             spi_pos8 = spiuint_to_float(Rxdate[36], -12.5, 12.5, 16);
             spi_vel8 = spiuint_to_float(Rxdate[37], -30.0f,30.0f, 12);
             spi_toq8 = spiuint_to_float(Rxdate[38], -10.0f, 10.0f, 12);
        }
        else
        {
            idd8=0;
            ++nnumid8;
        }
    }
    else
    {
         spi_pos8 = spiuint_to_float(Rxdate[35], -12.5, 12.5, 16);
         spi_vel8 = spiuint_to_float(Rxdate[36], -30.0f,30.0f, 12);
         spi_toq8 = spiuint_to_float(Rxdate[37], -10.0f, 10.0f, 12);	        
    }

    //lb leg_5
    motor_states[15].q = spi_pos0; 
    motor_states[15].dq = spi_vel0; 
    motor_states[15].tau = spi_toq0;

    motor_states[16].q = spi_pos1; 
    motor_states[16].dq = spi_vel1; 
    motor_states[16].tau = spi_toq1;

    motor_states[17].q = spi_pos2; 
    motor_states[17].dq = spi_vel2; 
    motor_states[17].tau = spi_toq2;

    //rb leg_4
    motor_states[12].q = spi_pos3; 
    motor_states[12].dq = spi_vel3; 
    motor_states[12].tau = spi_toq3;

    motor_states[13].q = spi_pos4; 
    motor_states[13].dq = spi_vel4; 
    motor_states[13].tau = spi_toq4;

    motor_states[14].q = spi_pos5; 
    motor_states[14].dq = spi_vel5; 
    motor_states[14].tau = spi_toq5;

    //rf leg_0
    motor_states[0].q = spi_pos6; 
    motor_states[0].dq = spi_vel6; 
    motor_states[0].tau = spi_toq6;

    motor_states[1].q = spi_pos7; 
    motor_states[1].dq = spi_vel7; 
    motor_states[1].tau = spi_toq7;

    motor_states[2].q = spi_pos8; 
    motor_states[2].dq = spi_vel8; 
    motor_states[2].tau = spi_toq8;

    //S1+123=标号6
    //S1+456=标号5
    ///S1+789=标号4
    // std::cout << "ld1: "<<" id: "<<std::hex << idd0 <<" pos: "<<std::fixed<< spi_pos0*_rad<<" vel: "<<std::fixed<<spi_vel0<<" toq: "<<std::fixed<<spi_toq0<<std::endl;
    // std::cout << "ld2: "<<" id: "<<std::hex << idd1 <<" pos: "<<std::fixed<< spi_pos1*_rad<<" vel: "<<std::fixed<<spi_vel1<<" toq: "<<std::fixed<<spi_toq1<<std::endl;
    // std::cout << "ld3: "<<" id: "<<std::hex << idd2 <<" pos: "<<std::fixed<< spi_pos2*_rad<<" vel: "<<std::fixed<<spi_vel2<<" toq: "<<std::fixed<<spi_toq2<<std::endl;
    // std::cout << "rd1: "<<" id: "<<std::hex << idd3 <<" pos: "<<std::fixed<< spi_pos3*_rad<<" vel: "<<std::fixed<<spi_vel3<<" toq: "<<std::fixed<<spi_toq3<<std::endl;
    // std::cout << "rd2: "<<" id: "<<std::hex << idd4 <<" pos: "<<std::fixed<< spi_pos4*_rad<<" vel: "<<std::fixed<<spi_vel4<<" toq: "<<std::fixed<<spi_toq4<<std::endl;
    // std::cout << "rd3: "<<" id: "<<std::hex << idd5 <<" pos: "<<std::fixed<< spi_pos5*_rad<<" vel: "<<std::fixed<<spi_vel5<<" toq: "<<std::fixed<<spi_toq5<<std::endl;
    // std::cout << "rf1: "<<" id: "<<std::hex << idd6 <<" pos: "<<std::fixed<< spi_pos6*_rad<<" vel: "<<std::fixed<<spi_vel6<<" toq: "<<std::fixed<<spi_toq6<<std::endl;
    // std::cout << "rf2: "<<" id: "<<std::hex << idd7 <<" pos: "<<std::fixed<< spi_pos7*_rad<<" vel: "<<std::fixed<<spi_vel7<<" toq: "<<std::fixed<<spi_toq7<<std::endl;
    // std::cout << "rf3: "<<" id: "<<std::hex << idd8 <<" pos: "<<std::fixed<< spi_pos8*_rad<<" vel: "<<std::fixed<<spi_vel8<<" toq: "<<std::fixed<<spi_toq8<<std::endl;
    // std::cout << "error: "<<" 1: "<<std::dec << nnumid0 <<" 2: "<<std::dec<< nnumid1<<" 3: "<<std::dec<<nnumid2
    //             <<" 4: "<<std::dec << nnumid3 <<" 5: "<<std::dec<< nnumid4<<" 6: "<<std::dec<<nnumid5
    //             <<" 7: "<<std::dec << nnumid6 <<" 8: "<<std::dec<< nnumid7<<" 9: "<<std::dec<<nnumid8
    //             <<std::endl;

    // printf("\n");       
}

// void spi_sr::pos_loaddate(float  pos0,float   pos1,float   pos2,float  pos3,float   pos4,float   pos5,float   pos6,float   pos7,float   pos8)
void spi_sr::pos_loaddate()
{
    float spi_KP=20;
    float spi_KD=1;
    float spi_trop=0;
    float spi_vel=0;
    uint16_t loadmag[46]={0};

    uint16_t spi_pos_tmp0=spifloat_to_uint(lb1.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp1=spifloat_to_uint(lb2.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp2=spifloat_to_uint(lb3.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp3=spifloat_to_uint(rb1.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp4=spifloat_to_uint(rb2.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp5=spifloat_to_uint(rb3.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp6=spifloat_to_uint(rf1.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp7=spifloat_to_uint(rf2.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp8=spifloat_to_uint(rf3.q,-12.5f,12.5f,16);

    uint16_t spi_vel_tmp0=spifloat_to_uint(lb1.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp1=spifloat_to_uint(lb2.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp2=spifloat_to_uint(lb3.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp3=spifloat_to_uint(rb1.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp4=spifloat_to_uint(rb2.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp5=spifloat_to_uint(rb3.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp6=spifloat_to_uint(rf1.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp7=spifloat_to_uint(rf2.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp8=spifloat_to_uint(rf3.dq,-30.0f,30.0f,12);

    uint16_t spi_kp_tmp0=spifloat_to_uint(lb1.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp1=spifloat_to_uint(lb2.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp2=spifloat_to_uint(lb3.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp3=spifloat_to_uint(rb1.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp4=spifloat_to_uint(rb2.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp5=spifloat_to_uint(rb3.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp6=spifloat_to_uint(rf1.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp7=spifloat_to_uint(rf2.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp8=spifloat_to_uint(rf3.Kp,0.0f,500.0f,12);

    uint16_t spi_kd_tmp0=spifloat_to_uint(lb1.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp1=spifloat_to_uint(lb2.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp2=spifloat_to_uint(lb3.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp3=spifloat_to_uint(rb1.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp4=spifloat_to_uint(rb2.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp5=spifloat_to_uint(rb3.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp6=spifloat_to_uint(rf1.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp7=spifloat_to_uint(rf2.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp8=spifloat_to_uint(rf3.Kd,0.0f,5.0f,12);

    uint16_t spi_tor_tmp0=spifloat_to_uint(lb1.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp1=spifloat_to_uint(lb2.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp2=spifloat_to_uint(lb3.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp3=spifloat_to_uint(rb1.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp4=spifloat_to_uint(rb2.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp5=spifloat_to_uint(rb3.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp6=spifloat_to_uint(rf1.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp7=spifloat_to_uint(rf2.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp8=spifloat_to_uint(rf3.tau,-10.0f,10.0f,12);
    loadmag[0] = 0x2255;

            loadmag[1]=spi_pos_tmp0;
            loadmag[2]=spi_vel_tmp0;
            loadmag[3]=spi_kp_tmp0;
            loadmag[4]=spi_kd_tmp0;
            loadmag[5]=spi_tor_tmp0;

            loadmag[6]=spi_pos_tmp1;
            loadmag[7]=spi_vel_tmp1;
            loadmag[8]=spi_kp_tmp1;
            loadmag[9]=spi_kd_tmp1;
            loadmag[10]=spi_tor_tmp1;

            loadmag[11]=spi_pos_tmp2;
            loadmag[12]=spi_vel_tmp2;
            loadmag[13]=spi_kp_tmp2;
            loadmag[14]=spi_kd_tmp2;
            loadmag[15]=spi_tor_tmp2;

            loadmag[16]=spi_pos_tmp3;
            loadmag[17]=spi_vel_tmp3;
            loadmag[18]=spi_kp_tmp3;
            loadmag[19]=spi_kd_tmp3;
            loadmag[20]=spi_tor_tmp3;

            loadmag[21]=spi_pos_tmp4;
            loadmag[22]=spi_vel_tmp4;
            loadmag[23]=spi_kp_tmp4;
            loadmag[24]=spi_kd_tmp4;
            loadmag[25]=spi_tor_tmp4;

            loadmag[26]=spi_pos_tmp5;
            loadmag[27]=spi_vel_tmp5;
            loadmag[28]=spi_kp_tmp5;
            loadmag[29]=spi_kd_tmp5;
            loadmag[30]=spi_tor_tmp5;

            loadmag[31]=spi_pos_tmp6;
            loadmag[32]=spi_vel_tmp6;
            loadmag[33]=spi_kp_tmp6;
            loadmag[34]=spi_kd_tmp6;
            loadmag[35]=spi_tor_tmp6;

            loadmag[36]=spi_pos_tmp7;
            loadmag[37]=spi_vel_tmp7;
            loadmag[38]=spi_kp_tmp7;
            loadmag[39]=spi_kd_tmp7;
            loadmag[40]=spi_tor_tmp7;

            loadmag[41]=spi_pos_tmp8;
            loadmag[42]=spi_vel_tmp8;
            loadmag[43]=spi_kp_tmp8;
            loadmag[44]=spi_kd_tmp8;
            loadmag[45]=spi_tor_tmp8;
        
        send_date(loadmag);//靠近电源侧S1	//靠近hzg附近//远离派镇

}

    // std::cout << "lm1: "<<" id: "<<std::hex << idd0 <<" pos: "<<std::fixed<< spi_pos0*_rad<<" vel: "<<std::fixed<<spi_vel0<<" toq: "<<std::fixed<<spi_toq0<<std::endl;
    // std::cout << "lm2: "<<" id: "<<std::hex << idd1 <<" pos: "<<std::fixed<< spi_pos1*_rad<<" vel: "<<std::fixed<<spi_vel1<<" toq: "<<std::fixed<<spi_toq1<<std::endl;
    // std::cout << "lm3: "<<" id: "<<std::hex << idd2 <<" pos: "<<std::fixed<< spi_pos2*_rad<<" vel: "<<std::fixed<<spi_vel2<<" toq: "<<std::fixed<<spi_toq2<<std::endl;
    // std::cout << "rm1: "<<" id: "<<std::hex << idd3 <<" pos: "<<std::fixed<< spi_pos3*_rad<<" vel: "<<std::fixed<<spi_vel3<<" toq: "<<std::fixed<<spi_toq3<<std::endl;
    // std::cout << "rm2: "<<" id: "<<std::hex << idd4 <<" pos: "<<std::fixed<< spi_pos4*_rad<<" vel: "<<std::fixed<<spi_vel4<<" toq: "<<std::fixed<<spi_toq4<<std::endl;
    // std::cout << "rm3: "<<" id: "<<std::hex << idd5 <<" pos: "<<std::fixed<< spi_pos5*_rad<<" vel: "<<std::fixed<<spi_vel5<<" toq: "<<std::fixed<<spi_toq5<<std::endl;
    // std::cout << "lf1: "<<" id: "<<std::hex << idd6 <<" pos: "<<std::fixed<< spi_pos6*_rad<<" vel: "<<std::fixed<<spi_vel6<<" toq: "<<std::fixed<<spi_toq6<<std::endl;
    // std::cout << "lf2: "<<" id: "<<std::hex << idd7 <<" pos: "<<std::fixed<< spi_pos7*_rad<<" vel: "<<std::fixed<<spi_vel7<<" toq: "<<std::fixed<<spi_toq7<<std::endl;
    // std::cout << "lf3: "<<" id: "<<std::hex << idd8 <<" pos: "<<std::fixed<< spi_pos8*_rad<<" vel: "<<std::fixed<<spi_vel8<<" toq: "<<std::fixed<<spi_toq8<<std::endl;
    // std::cout << "error: "<<" 1: "<<std::dec << numid0 <<" 2: "<<std::dec<< numid1<<" 3: "<<std::dec<<numid2
    //             <<" 4: "<<std::dec << numid3 <<" 5: "<<std::dec<< numid4<<" 6: "<<std::dec<<numid5
    //             <<" 7: "<<std::dec << numid6 <<" 8: "<<std::dec<< numid7<<" 9: "<<std::dec<<numid8
    //             <<std::endl;
// void spi_sr::pos_loaddate2(float  pos0,float   pos1,float   pos2,float  pos3,float   pos4,float   pos5,float   pos6,float   pos7,float   pos8)
void spi_sr::pos_loaddate2()
{
    float spi_KP=20;
    float spi_KD=1;
    float spi_trop=0;
    float spi_vel=0;
    uint16_t loadmag2[46]={0};

    uint16_t spi_pos_tmp0=spifloat_to_uint(lm1.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp1=spifloat_to_uint(lm2.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp2=spifloat_to_uint(lm3.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp3=spifloat_to_uint(rm1.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp4=spifloat_to_uint(rm2.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp5=spifloat_to_uint(rm3.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp6=spifloat_to_uint(lf1.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp7=spifloat_to_uint(lf2.q,-12.5f,12.5f,16);
    uint16_t spi_pos_tmp8=spifloat_to_uint(lf3.q,-12.5f,12.5f,16);

    uint16_t spi_vel_tmp0=spifloat_to_uint(lm1.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp1=spifloat_to_uint(lm2.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp2=spifloat_to_uint(lm3.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp3=spifloat_to_uint(rm1.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp4=spifloat_to_uint(rm2.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp5=spifloat_to_uint(rm3.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp6=spifloat_to_uint(lf1.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp7=spifloat_to_uint(lf2.dq,-30.0f,30.0f,12);
    uint16_t spi_vel_tmp8=spifloat_to_uint(lf3.dq,-30.0f,30.0f,12);

    uint16_t spi_kp_tmp0=spifloat_to_uint(lm1.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp1=spifloat_to_uint(lm2.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp2=spifloat_to_uint(lm3.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp3=spifloat_to_uint(rm1.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp4=spifloat_to_uint(rm2.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp5=spifloat_to_uint(rm3.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp6=spifloat_to_uint(lf1.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp7=spifloat_to_uint(lf2.Kp,0.0f,500.0f,12);
    uint16_t spi_kp_tmp8=spifloat_to_uint(lf3.Kp,0.0f,500.0f,12);

    uint16_t spi_kd_tmp0=spifloat_to_uint(lm1.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp1=spifloat_to_uint(lm2.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp2=spifloat_to_uint(lm3.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp3=spifloat_to_uint(rm1.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp4=spifloat_to_uint(rm2.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp5=spifloat_to_uint(rm3.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp6=spifloat_to_uint(lf1.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp7=spifloat_to_uint(lf2.Kd,0.0f,5.0f,12);
    uint16_t spi_kd_tmp8=spifloat_to_uint(lf3.Kd,0.0f,5.0f,12);

    uint16_t spi_tor_tmp0=spifloat_to_uint(lm1.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp1=spifloat_to_uint(lm2.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp2=spifloat_to_uint(lm3.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp3=spifloat_to_uint(rm1.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp4=spifloat_to_uint(rm2.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp5=spifloat_to_uint(rm3.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp6=spifloat_to_uint(lf1.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp7=spifloat_to_uint(lf2.tau,-10.0f,10.0f,12);
    uint16_t spi_tor_tmp8=spifloat_to_uint(lf3.tau,-10.0f,10.0f,12);

        loadmag2[0]=0x2244;

            loadmag2[1] = spi_pos_tmp0;
            loadmag2[2] = spi_vel_tmp0;
            loadmag2[3] = spi_kp_tmp0;
            loadmag2[4] = spi_kd_tmp0;
            loadmag2[5] = spi_tor_tmp0;

            loadmag2[6] = spi_pos_tmp1;
            loadmag2[7] = spi_vel_tmp1;
            loadmag2[8] = spi_kp_tmp1;
            loadmag2[9] = spi_kd_tmp1;
            loadmag2[10] = spi_tor_tmp1;

            loadmag2[11] = spi_pos_tmp2;
            loadmag2[12] = spi_vel_tmp2;
            loadmag2[13] = spi_kp_tmp2;
            loadmag2[14] = spi_kd_tmp2;
            loadmag2[15] = spi_tor_tmp2;

            loadmag2[16] = spi_pos_tmp3;
            loadmag2[17] = spi_vel_tmp3;
            loadmag2[18] = spi_kp_tmp3;
            loadmag2[19] = spi_kd_tmp3;
            loadmag2[20] = spi_tor_tmp3;

            loadmag2[21] = spi_pos_tmp4;
            loadmag2[22] = spi_vel_tmp4;
            loadmag2[23] = spi_kp_tmp4;
            loadmag2[24] = spi_kd_tmp4;
            loadmag2[25] = spi_tor_tmp4;

            loadmag2[26] =spi_pos_tmp5;
            loadmag2[27] =spi_vel_tmp5;
            loadmag2[28] =spi_kp_tmp5;
            loadmag2[29] =spi_kd_tmp5;
            loadmag2[30] =spi_tor_tmp5;

            loadmag2[31]=spi_pos_tmp6;
            loadmag2[32]=spi_vel_tmp6;
            loadmag2[33]=spi_kp_tmp6;
            loadmag2[34]=spi_kd_tmp6;
            loadmag2[35]=spi_tor_tmp6;

            loadmag2[36]=spi_pos_tmp7;
            loadmag2[37]=spi_vel_tmp7;
            loadmag2[38]=spi_kp_tmp7;
            loadmag2[39]=spi_kd_tmp7;
            loadmag2[40]=spi_tor_tmp7;

            loadmag2[41]=spi_pos_tmp8;
            loadmag2[42]=spi_vel_tmp8;
            loadmag2[43]=spi_kp_tmp8;
            loadmag2[44]=spi_kd_tmp8;
            loadmag2[45]=spi_tor_tmp8;
        
            send_date2(loadmag2);
}

void spi_sr::enter_close_loop(){
    send_date(motor_enable0);//靠近电源侧S1	//靠近hzg附近//远离派镇
    usleep(500);
    send_date2(motor_enable1);//远离电源侧S2	//靠近电源侧
    usleep(500);
}
void spi_sr::exit_close_loop(){
    send_date(motor_disable0);//靠近电源侧S1	//靠近hzg附近//远离派镇
    usleep(500);
    send_date2(motor_disable1);//远离电源侧S2	//靠近电源侧
    usleep(500);
}

void spi_sr::load_all_motorCmd(){
//     MotorCmd rf1,rf2,rf3;
//     MotorCmd rm1,rm2,rm3;
//     MotorCmd rb1,rb2,rb3;
//     MotorCmd lf1,lf2,lf3;
//     MotorCmd lm1,lm2,lm3;
//     MotorCmd lb1,lb2,lb3;
}

void spi_sr::send_all_data(){
    pos_loaddate();
    usleep(500);
    pos_loaddate2();
    // usleep(2000);
}
