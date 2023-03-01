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
	unsigned int sequenceNumber = 0;    
	unsigned int sampleCounter = 0;  
 	unsigned int status = 0;		
	int32 fx = 0;
	int32 fy = 0;
	int32 fz = 0;
	int32 tx = 0;
	int32 ty = 0;
	int32 tz = 0;
} Response;

class OnRobotForceTorqueSensor
{
public:

    static std::shared_ptr<OnRobotForceTorqueSensor> getInstance(
        std::string ipAddress = "192.168.1.1", 
        bool fake_mode = false,
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
    void startStreaming(bool verbose=false);
    void stopStreaming();
    
    void getLatestDataDArray(std::array<double, 7> &data_darr);
    void getLatestDataVec(std::vector<double> &data_dvec);
    
    bool isFake();

private:
    
    static std::map<std::string, std::weak_ptr<OnRobotForceTorqueSensor>> instances;
    
    OnRobotForceTorqueSensor(std::string ipAddress, int32 sampleingHz, int32 filterType, bool enableBiasing, bool fake_mode);

    int openDevice(const char * ipAddress, uint16 port);
    int closeDevice() { close(handle_); return 1;}

    void sendCommand(uint16 command, uint32 data);

    void showResponse(Response r);

    bool fake_mode_ = false;

    SOCKET_HANDLE handle_;

    std::string ipAddress;
    int32 samplingHz=100;
    int32 samplingDt_ms=10;
    // int32 samplingDt_us=10000;

    bool verbose_ = false;

    // rx thread 
    std::array<double, 7> data_buf_ = {0., 0., 0., 0., 0., 0., 0.};
    Response receive();
    static void* static_rx_thread(void* pThis);
    void rx_thread();
    std::shared_timed_mutex rx_lck_;
    bool rx_stop_ = false;
    pthread_t threadid_;
};