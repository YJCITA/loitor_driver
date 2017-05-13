#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "math.h"

#define IMU_FRAME_LEN 32
#include "loitorimu.h"

bool shut_down_imu=false;

timeval visensor_startTime;		// time-stamp
struct timeval imu_SysTime;

float imu_time=0;
bool visensor_imu_updated=false;

bool visensor_query_imu_update()
{
	return visensor_imu_updated;
}
bool visensor_mark_imu_update()
{
	visensor_imu_updated=true;
}
bool visensor_erase_imu_update()
{
	visensor_imu_updated=false;
}

int visensor_set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if ( tcgetattr( fd,&oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':
        newtio.c_cflag &= ~PARENB;
        break;
    }

    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 32;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

int visensor_open_port(const char* dev_str)
{
    long vdisable;

    int fd = open( dev_str, O_RDWR|O_NOCTTY|O_NDELAY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return(-1);
    }

    if(fcntl(fd, F_SETFL, 0)<0)
        printf("fcntl failed!\n");
    else
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    if(isatty(STDIN_FILENO)==0)
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");
    printf("fd-open=%d\n",fd);
    return fd;
}

static int find_55aa(unsigned char* buf,int len)
{
    int i;
    for(i=0; i<len-1; i++)
    {
        if(buf[i]==0x55)
            if(buf[i+1]==0xAA)
                return i;
    }
    if(buf[len-1]==0x55)
        return -1;
    return -2;
}


int visensor_send_imu_frame(int fd, unsigned char* data, int len)
{
    return write(fd,data,len);
}

#include <sys/time.h>
#include <boost/iterator/iterator_concepts.hpp>

int visensor_get_imu_frame(int fd, unsigned char* imu_frame)
{
    unsigned char imu_frame_buf[2*IMU_FRAME_LEN];
    memset(imu_frame,0,IMU_FRAME_LEN);
    memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
    int num_get = 0;

    static int flush_count = 0;
    flush_count = (flush_count+1)%200;
    if(flush_count==0)
        tcflush(fd,TCIFLUSH);
	
    static struct timeval getIMUTime;
	struct timeval endTime;
	float fTimeuse;
	int valid_len = 0;
    while(!shut_down_imu){
        //读到32个以上字节
// 		printf("first read, valid_len: %d, num_get_pre: %d\n", valid_len, num_get);
        while(num_get<IMU_FRAME_LEN){
			// 这个是一次性读空buffer中的数据
            int temp_read_get = read(fd,&imu_frame_buf[num_get],IMU_FRAME_LEN-num_get);
            if(temp_read_get<=0){
				printf("1\n");
                continue;//读取错误，不得往num_get上加
            }
            num_get+=temp_read_get;//实际读到了num_get字节数
        }
//         printf("num_get_cur: %d\n", num_get);
        
        gettimeofday(&getIMUTime, NULL);
		gettimeofday(&endTime, NULL);
		gettimeofday(&imu_SysTime, NULL);
		imu_SysTime.tv_usec -= 3500;
		if(imu_SysTime.tv_usec < 0){
			imu_SysTime.tv_usec += 1000000;
			imu_SysTime.tv_sec -= 1;
		}

        //查询55AA的位置
        int position_55aa = find_55aa(imu_frame_buf,num_get);
        //如果查不到55AA(-2)，末尾也没有55，那么此次读取失败，清空imu_frame_buf,从头再来
        if(position_55aa == -2){
            memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
            num_get = 0;
			printf("2- read 55aa failed \n");
            continue;
        }
        
        //如果末尾是55，那么清空imu_frame_buf,然后将首位置为55
        if(position_55aa == -1) {
            memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
            num_get = 1;
            imu_frame_buf[0] = 0x55;
			printf("3\n");
            continue;
        }
        
        //如果找到了55AA，那么首先检验有效帧长度
        valid_len = num_get - position_55aa;
		
        //如果有效帧长度不足32，那么将数据移动到帧头，然后继续读取剩余帧
        if(valid_len<IMU_FRAME_LEN){
            for(int i=0; i<valid_len; i++)
                imu_frame_buf[i] = imu_frame_buf[position_55aa+i];
            for(int i=valid_len; i<2*IMU_FRAME_LEN; i++)
                imu_frame_buf[i] = 0;
            num_get = valid_len;
			printf("4， valid_len：%d, num_get: %d, position_55aa: %d\n", valid_len, num_get, position_55aa);
            continue;
        }
        //如果有效帧长度达到32，那么检查校验和
        unsigned char checksum = 0;
        for(int i=2; i<IMU_FRAME_LEN-1; i++)
			checksum += imu_frame_buf[position_55aa+i];
		
        //如果校验和不正确，那么清除该帧头和之前的所有数据，然后将剩余数据移动到帧头，再继续读取剩余数据
        if(checksum != imu_frame_buf[position_55aa+IMU_FRAME_LEN-1]){
            for(int i=2; i<valid_len; i++)
                imu_frame_buf[i-2] = imu_frame_buf[position_55aa+i];
            for(int i=valid_len-2; i<2*IMU_FRAME_LEN; i++)
                imu_frame_buf[i] = 0;
            num_get = valid_len-2;
			valid_len = 0;
			printf("5\n");
            continue;
        }
        //如果校验和正确，而有效帧长度超过了32，那么说明已经有数据溢出，应立即清除串口缓存,然后重新开始读取
        if(valid_len>IMU_FRAME_LEN)
        {
            tcflush(fd,TCIFLUSH);
            memset(imu_frame_buf,0,2*IMU_FRAME_LEN);
			printf("6- valid_len>IMU_FRAME_LEN:valid_len: %d is full \n", valid_len);
            num_get = 0;
			valid_len = 0;
            continue;
        }
        //如果校验和正确，那么输出该帧数据
        memcpy(imu_frame, &imu_frame_buf[position_55aa], IMU_FRAME_LEN);
        static float time_interval = 5000;
        static float last_time,new_time,new_pred,last_pred;
        new_time = getIMUTime.tv_usec;
        int newtime_interval;
        newtime_interval = new_time-last_time;
        if(newtime_interval<4000 || newtime_interval>6000)
            newtime_interval = 5000;
        time_interval = time_interval*0.999+0.001*newtime_interval;
        if(new_time-last_pred<time_interval)
            new_pred = new_time;
        else
            new_pred = last_pred + time_interval + 0.001*(new_time - last_pred - time_interval);
        last_time = new_time;
        last_pred = new_pred;

		fTimeuse = (endTime.tv_sec - visensor_startTime.tv_sec) + 0.000001*(endTime.tv_usec - visensor_startTime.tv_usec);
		imu_time = fTimeuse;
        break;
    }

    return 0;
}

float data_norm(float * q,int length)
{
    float datasum=0.0;
    for(int i=0; i<length; i++)datasum+=q[i]*q[i];
    return sqrt(datasum);
}
int q_normalize(float* q)
{
    float qnorm = data_norm(q,4);
    for(int i=0; i<4; i++)q[i]/=qnorm;
}

int q_mult(float* q1,float* q2,float* qout)
{
    qout[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
    qout[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
    qout[2]=q1[0]*q2[2]+q1[2]*q2[0]+q1[3]*q2[1]-q1[1]*q2[3];
    qout[3]=q1[0]*q2[3]+q1[3]*q2[0]+q1[1]*q2[2]-q1[2]*q2[1];
}

int q_rotvec(float* q,float* rin,float* rout)
{
    float q_rin[4]= {0,rin[0],rin[1],rin[2]};
    float q_temp[4];
    q_mult(q,q_rin,q_temp);
    float q_inv[4]= {q[0],-q[1],-q[2],-q[3]};
    float q_temp_out[4];
    q_mult(q_temp,q_inv,q_temp_out);
    for(int i=0; i<3; i++)rout[i]=q_temp_out[i+1];
}


int g_imu_num_pre = 0;
timeval imu_system_time_pre;
timeval serial_frame_system_time_cur, serial_frame_system_time_pre;
bool g_is_first_read_imu = true;
bool g_is_read_imu_init_ok = false;
int g_index_in_serial_frame = 0; // 在串口帧中（16个IMU数据）的IMU的序号
int g_imu_num_pre_serial_frame = 0;  //上一帧serial来的时候数据

int format_imu_systemtime(const unsigned char* imu_frame, float& imu_time, timeval& imu_system_time)
{
	int imu_num_cur = (int)imu_frame[2];
	if(g_is_first_read_imu){
		imu_system_time_pre = imu_SysTime;
		g_imu_num_pre = imu_num_cur;
		g_is_first_read_imu = false;
		g_imu_num_pre_serial_frame = imu_num_cur;
	}
	// 对IMU读取到的时间进行整定
	if(!g_is_read_imu_init_ok){
		// 本质上  最初始的几个数据 基本是没法获取准确的绝对时间的，所以临时处理成当时读到数据的时间
		double dt_imu_t = (imu_SysTime.tv_sec - imu_system_time_pre.tv_sec)*1e3 + (imu_SysTime.tv_usec - imu_system_time_pre.tv_usec)/1e3;	
		if(dt_imu_t > 60){
			g_index_in_serial_frame = 0;
			serial_frame_system_time_cur = imu_SysTime;
			int tmp_us = serial_frame_system_time_cur.tv_usec - (16-1)*5000;
			if(tmp_us < 0){
				serial_frame_system_time_cur.tv_sec -= 1;
				serial_frame_system_time_cur.tv_usec = tmp_us + 1e6;
			}else{
				serial_frame_system_time_cur.tv_usec = tmp_us;
			}
			
			serial_frame_system_time_pre = imu_SysTime;
			g_imu_num_pre_serial_frame = imu_num_cur;
			g_is_read_imu_init_ok = true;
		}
		
		imu_time = imu_time;
		imu_system_time = imu_SysTime;
	}else{
		// 前后两次读到数据的时间差， 因为loitor端应该是通过串口按16帧IMU数据组成一个串口包，发出来
		// 所以，每隔16个数据，dt就会有一个比较多大的dt（差不多应该是1000ms/200*16=80ms, 但是因为每次数据imu_data_feed读完一次32字节，就对delay 0.5ms）
		// 所以最后的：dt_max = 80ms - 0.5*16 = 72ms, 不过实际可能会波动
		double dt_imu_t = (imu_SysTime.tv_sec - imu_system_time_pre.tv_sec)*1e3 + (imu_SysTime.tv_usec - imu_system_time_pre.tv_usec)/1e3;	
		if(dt_imu_t > 60){
			double dt_serial = (imu_SysTime.tv_sec - serial_frame_system_time_pre.tv_sec)*1e3 + (imu_SysTime.tv_usec - serial_frame_system_time_pre.tv_usec)/1e3;
// 			printf("----------dt_serial: %f\n", dt_serial);
			
			int d_imu_num_serial_frame = imu_num_cur - g_imu_num_pre_serial_frame; // 当前serial帧中IMU的数量
			if(d_imu_num_serial_frame < 0)
				d_imu_num_serial_frame += 200;
			
			serial_frame_system_time_cur = serial_frame_system_time_pre;
			
			printf("------11111----d_imu_num_serial_frame: %d\n", d_imu_num_serial_frame);
			g_index_in_serial_frame = 0;
			g_imu_num_pre_serial_frame = imu_num_cur;
			serial_frame_system_time_pre = imu_SysTime;
		}
		
		timeval imu_system_time_fix = serial_frame_system_time_cur;
		int d_imu_num = imu_num_cur - g_imu_num_pre;
		if(d_imu_num < 0)
			d_imu_num += 200;
// 		printf("----------d_imu_num: %d\n", d_imu_num);
		g_index_in_serial_frame += d_imu_num; // 根据当前读到imu和上一次读到的imu的num的差值，确定跟上一个IMU的dt=d_NUM*T  其中T=5ms
		printf("--1--g_index_in_serial_frame: %d\n", g_index_in_serial_frame);
		int tmp_us = serial_frame_system_time_cur.tv_usec + g_index_in_serial_frame*5000;
		if(tmp_us >= 1e6){
			imu_system_time_fix.tv_sec += 1;
			imu_system_time_fix.tv_usec = tmp_us - 1e6;
		}else{
			imu_system_time_fix.tv_usec = tmp_us;
		}
		
		imu_time = imu_time;
		imu_system_time = imu_system_time_fix;
	}
	
	imu_system_time_pre = imu_SysTime;
	g_imu_num_pre = imu_num_cur;
}

int visensor_get_imu_data(unsigned char* imu_frame, short int* acc_offset, visensor_imudata *imudata_struct, bool show_data=false)
{	
	int imu_num_cur = (int)imu_frame[2];
	
	format_imu_systemtime(imu_frame, imudata_struct->imu_time, imudata_struct->system_time);
	
    int imu_data[10];
    imu_data[0] = *(short*)(&imu_frame[3]);
    imu_data[1] = *(short*)(&imu_frame[5]);
    imu_data[2] = *(short*)(&imu_frame[7]);

    imu_data[3] = *(short*)(&imu_frame[9]);
    imu_data[4] = *(short*)(&imu_frame[11]);
    imu_data[5] = *(short*)(&imu_frame[13]);

    imu_data[6] = *(int*)(&imu_frame[15]);
    imu_data[7] = *(int*)(&imu_frame[19]);
    imu_data[8] = *(int*)(&imu_frame[23]);
    imu_data[9] = *(int*)(&imu_frame[27]);

	imudata_struct->rx = 1.0*imu_data[0]/32768*2000; 
	imudata_struct->ry = 1.0*imu_data[1]/32768*2000; 
	imudata_struct->rz = 1.0*imu_data[2]/32768*2000; 
    float facc_nobias[3]= {imu_data[3]-acc_offset[0],imu_data[4]-acc_offset[1],imu_data[5]-acc_offset[2]};
	
	imudata_struct->ax = 1.0*facc_nobias[0]/16384*9.81; 
	imudata_struct->ay = 1.0*facc_nobias[1]/16384*9.81; 
	imudata_struct->az = 1.0*facc_nobias[2]/16384*9.81;

    float quat[4]= {imu_data[6],imu_data[7],imu_data[8],imu_data[9]};
    float facc[3]= {imu_data[3],imu_data[4],imu_data[5]};

    static float faccg[3]= {0,0,0};
    q_normalize(quat);
	imudata_struct->qw = quat[0];
	imudata_struct->qx = quat[1];
	imudata_struct->qy = quat[2];
	imudata_struct->qz = quat[3];
	imudata_struct->num = imu_frame[2];

    q_rotvec(quat, facc_nobias, faccg);

	if(show_data){
		printf("Num: %03d",imu_frame[2]);
		printf("Gyr: %6d,%6d,%6d",imu_data[0],imu_data[1],imu_data[2]);
		printf("Acc: %6d,%6d,%6d",(int)facc[0],(int)facc[1],(int)facc[2]);
		printf("Quat: %11f,%11f,%11f,%11f\r\n",quat[0],quat[1],quat[2],quat[3]);
	}
}
