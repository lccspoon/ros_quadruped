#include "interface/imu.h"

#define REFRESH_INTERVAL 20000 // 以微秒为单位，对应于50Hz
hipnuc_raw_t hipnuc_raw = {0};
int baud_rate = 921600;
uint8_t recv_buf[2048];
char log[512];
int len;
const char *port_name = "ttyUSB0";
int fd;

bool imu_launch_first = false;
int imu_run(void)
{
	if( imu_launch_first == false ){

		fd = serial_port_open(port_name);
		if (fd < 0)
			return 1;
		if (serial_port_configure(fd, baud_rate) < 0)
		{
			fprintf(stderr, "Cannot open %s\n", port_name);
			serial_port_close(fd);
			return 1;
		}
		serial_send_then_recv(fd, "AT+EOUT=1\r\n", "OK\r\n", reinterpret_cast<char*>(recv_buf), sizeof(recv_buf), 200);
		// printf("Being read data...\n");

	}
	imu_launch_first = true;

	printf("Imu ready!\n");
	while (1)
	{
		len = read(fd, recv_buf, sizeof(recv_buf));

		for (int i = 0; i < len; i++)
		{
			if (hipnuc_input(&hipnuc_raw, recv_buf[i]))
			{
				hipnuc_dump_packet(&hipnuc_raw, log, sizeof(log));
				// printf("\033[H\033[J");
				// printf("%s", log);
			}
		}
		// usleep(REFRESH_INTERVAL);
		usleep(3000);
	}
}
