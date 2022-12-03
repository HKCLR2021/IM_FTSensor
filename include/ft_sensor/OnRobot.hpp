#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <pthread.h>
#include <vector>
#include <shared_mutex>

typedef int SOCKET_HANDLE;

#define PORT			49152	/* Port the Ethernet DAQ always uses */
#define SAMPLE_COUNT	10		/* 10 incoming samples */
#define SPEED			10		/* 1000 / SPEED = Speed in Hz */
#define FILTER			4		/* 0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz */
#define BIASING_ON		0xFF    /* Biasing on */
#define BIASING_OFF		0x00    /* Biasing off */

#define COMMAND_START	0x0002  /* Command for start streaming */
#define COMMAND_STOP	0x0000  /* Command for stop streaming */
#define COMMAND_BIAS	0x0042  /* Command for toggle biasing */
#define COMMAND_FILTER	0x0081  /* Command for setting filter */
#define COMMAND_SPEED	0x0082  /* Command for setting speed */


#define		UNIT  1 // 0 - Dimensionless  | 1 - Newton/Newton-meter

#if UNIT == 1
#define		FORCE_DIV	10000.0  // Default divide value
#define		TORQUE_DIV	100000.0 // Default divide value
#else
#define FORCE_DIV	1.0
#define TORQUE_DIV	1.0
#endif

typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

typedef struct ResponseStruct {
	unsigned int sequenceNumber;    
	unsigned int sampleCounter;  
 	unsigned int status;		
	int32 fx;
	int32 fy;
	int32 fz;
	int32 tx;
	int32 ty;
	int32 tz;
} Response;

class OnRobot
{
public:
    OnRobot();
    ~OnRobot() {};

    static OnRobot &getInstance()
    {
        static OnRobot instance;
        return instance;
    }
    OnRobot(OnRobot const &) = delete;
    void operator=(OnRobot const &) = delete;

    // Init & setting related stuff
    int openDevice(const char * ipAddress, uint16 port);
    bool disconnect() { close(handle_); }

    bool setSamplingRate(int32 frequency);
    bool setFilterType(int32 frequency);
    bool enableBiasing(bool biasing_on);

    // Data receving
    void startStreaming();
    void stopStreaming();
    void showResponse(Response r);
    
    void getLatestDataDArray(std::array<double, 6> &data_darr);
    void getLatestDataVec(std::vector<double> &data_dvec);

private:
    SOCKET_HANDLE handle_;
    void sendCommand(uint16 command, uint32 data);

    // rx thread 
    std::array<double, 6> data_buf_;
    Response receive();
    static void* static_rx_thread(void* pThis);
    void rx_thread();
    std::shared_timed_mutex rx_lck_;
    bool rx_stop_ = false;
    pthread_t threadid_;
};