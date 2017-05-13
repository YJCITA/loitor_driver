#ifndef LOITORIMU_H
#define LOITORIMU_H

#include <sys/time.h>

typedef struct 
{
	float imu_time;		// time-stamp
	timeval system_time;
	unsigned char num;
	float rx,ry,rz;
	float ax,ay,az;
	float qw,qx,qy,qz;
	//int timestamp;
}visensor_imudata;

extern timeval visensor_startTime;

bool visensor_query_imu_update();
bool visensor_mark_imu_update();
bool visensor_erase_imu_update();
int visensor_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
int visensor_open_port(const char* dev_str);
int visensor_get_imu_frame(int fd, unsigned char* imu_frame);
int visensor_get_imu_data(unsigned char* imu_frame,short int* acc_offset,visensor_imudata *imudata_struct,bool show_data);
int visensor_send_imu_frame(int fd, unsigned char* data, int len);

int print_imu_datawithtime(int data_count, float* fgyr, float* facc, double timestamp, double camtimes, short int* acc_offset);
int get_imu_datawithtime(int fd, int* data_count, float* gyr, float* acc, double* timestamp, double* camtimes);

int format_imu_systemtime(const unsigned char* imu_frame, float& imu_time, timeval& imu_system_time);

#endif
