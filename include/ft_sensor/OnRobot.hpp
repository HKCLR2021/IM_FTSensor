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
#include <map>
#include <memory>

typedef int SOCKET_HANDLE;

#define PORT			49152	/* Port the Ethernet DAQ always uses */
#define SAMPLE_COUNT	10		/* 10 incoming samples */
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

class OnRobotForceTorqueSensor
{
public:

    static std::shared_ptr<OnRobotForceTorqueSensor> getInstance(
        std::string ipAddress = "192.168.1.1", 
        int32 sampleingHz = 100, 
        int32 filterType = 4, 
        bool enableBiasing = true
    );

    ~OnRobotForceTorqueSensor();

    OnRobotForceTorqueSensor(OnRobotForceTorqueSensor const &) = delete;
    void operator=(OnRobotForceTorqueSensor const &) = delete;

    // setting related stuff
    bool setSamplingRate(int32 samplingHz);
    bool setFilterType(int32 filter_type);
    bool setEnableBiasing(bool biasing_on);

    int32 getSamplingRate() {return samplingHz;}

    // Data receving
    void startStreaming();
    void stopStreaming();
    
    void getLatestDataDArray(std::array<double, 6> &data_darr);
    void getLatestDataVec(std::vector<double> &data_dvec);

private:
    
    static std::map<std::string, std::weak_ptr<OnRobotForceTorqueSensor>> instances;
    
    OnRobotForceTorqueSensor(std::string ipAddress, int32 sampleingHz, int32 filterType, bool enableBiasing);

    int openDevice(const char * ipAddress, uint16 port);
    int closeDevice() { close(handle_); return 1;}

    void sendCommand(uint16 command, uint32 data);

    void showResponse(Response r);

    SOCKET_HANDLE handle_;

    int32 samplingHz=100;
    int32 samplingDt_ms=10;
    int32 samplingDt_us=10000;

    // rx thread 
    std::array<double, 6> data_buf_;
    Response receive();
    static void* static_rx_thread(void* pThis);
    void rx_thread();
    std::shared_timed_mutex rx_lck_;
    bool rx_stop_ = false;
    pthread_t threadid_;
};