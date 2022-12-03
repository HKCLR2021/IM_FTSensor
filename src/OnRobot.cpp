#include "ft_sensor/OnRobot.hpp"

OnRobot::OnRobot()
{
    handle_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (handle_ == -1) { 
		fprintf(stderr, "Error, Socket could not be opened.\n");
	}

    // Connect sensor
    const char *ipAddress = "192.168.1.1";
    uint16 port = PORT;
    openDevice(ipAddress, port);

    printf("OnRobot FT Sensor connected, Initializing...\n");

    // Init with default settings
    setSamplingRate(100); // 100Hz
    setFilterType(4); // 15Hz LPF
    enableBiasing(true); // biasing on
    
    printf("OnRobot Initialization done.\n");
}


int OnRobot::openDevice(const char * ipAddress, uint16 port)
{
	struct sockaddr_in addr;	
	struct hostent *he;	
	int err;

	he = gethostbyname(ipAddress);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	
	err = connect(handle_, (struct sockaddr *)&addr, sizeof(addr));
	if (err < 0) {
		return -3;
	}
	return 0;
}

Response OnRobot::receive()
{
    byte inBuffer[36];
	Response response;
	unsigned int uItems = 0;
	recv(handle_, (char *)inBuffer, 36, 0 );
	response.sequenceNumber = ntohl(*(uint32*)&inBuffer[0]);
	response.sampleCounter = ntohl(*(uint32*)&inBuffer[4]);
	response.status = ntohl(*(uint32*)&inBuffer[8]);
	response.fx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.fy = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4])); 
	response.fz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.tx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.ty = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	response.tz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
	return response;
}

void OnRobot::getLatestDataDArray(std::array<double, 6> &data_darr)
{
    rx_lck_.lock();
    std::copy(std::begin(data_buf_), std::end(data_buf_), std::begin(data_darr) );
    rx_lck_.unlock();
}


void OnRobot::getLatestDataVec(std::vector<double> &data_dvec)
{
    rx_lck_.lock();
    data_dvec = std::vector<double> (std::begin(data_buf_), std::end(data_buf_) );
    rx_lck_.unlock();
}


void OnRobot::showResponse(Response r)
{
    double fx = r.fx / FORCE_DIV;
	double fy = r.fy / FORCE_DIV;
	double fz = r.fz / FORCE_DIV;
	double tx = r.tx / TORQUE_DIV;
	double ty = r.ty / TORQUE_DIV;
	double tz = r.tz / TORQUE_DIV;
    #if UNIT == 1
        fprintf(stdout, "\nS:%u SN: %u SC: %u Fx: %.2f N Fy: %.2f N Fz: %.2f N Tx: %.2f Nm Ty: %.2f Nm Tz: %.2f Nm\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz);
    #else 
        fprintf(stdout, "\nS:%u SN: %u SC: %u Fx: %.2f Fy: %.2f Fz: %.2f Tx: %.2f Ty: %.2f Tz: %.2f\r\n", r.status, r.sequenceNumber, r.sampleCounter, fx, fy, fz, tx, ty, tz);
    #endif
        fflush(stdout);
}

bool OnRobot::setSamplingRate(int32 frequency)
{
    /* 1000 / SPEED = Speed in Hz */
    uint32 cmd_speed = 1000 / frequency;
    sendCommand(COMMAND_SPEED, cmd_speed);
}

bool OnRobot::setFilterType(int32 filter_type)
{
    /*  0 = No filter; 
        1 = 500 Hz; 
        2 = 150 Hz; 
        3 = 50 Hz; 
        4 = 15 Hz; 
        5 = 5 Hz; 
        6 = 1.5 Hz */
    sendCommand(COMMAND_FILTER, filter_type);
}

bool OnRobot::enableBiasing(bool biasing_on)
{
    uint32 cmd_bias = biasing_on ? 0xFF : 0X00;
    sendCommand(COMMAND_BIAS, cmd_bias);
    return true;
}

void* OnRobot::static_rx_thread(void* pThis)
{
    static_cast<OnRobot*>(pThis)->rx_thread();
}

void OnRobot::rx_thread()
{
    while (!rx_stop_)
    {
        sendCommand(COMMAND_START, SAMPLE_COUNT);
        for (int i = 0; i < SAMPLE_COUNT; ++i)
        {
            Response r = receive();
            // showResponse(r); // logging data
            
            rx_lck_.lock();
            // Data formatting
            double fx = r.fx / FORCE_DIV;
            double fy = r.fy / FORCE_DIV;
            double fz = r.fz / FORCE_DIV;
            double tx = r.tx / TORQUE_DIV;
            double ty = r.ty / TORQUE_DIV;
            double tz = r.tz / TORQUE_DIV;
            data_buf_ = {fx, fy, fz, tx, ty, tz};
            rx_lck_.unlock();

            usleep(10000); // wait for 10ms
        }
    }

    pthread_exit(0);
}

void OnRobot::startStreaming()
{
    rx_stop_ = false;
    if (pthread_create(&threadid_, NULL, static_rx_thread, (void*)this ) != 0 )    
    {
        printf("start streaming failed\n");
        pthread_detach(threadid_);
    }

}

void OnRobot::stopStreaming()
{
    rx_stop_ = true;
}

void OnRobot::sendCommand(uint16 command, uint32 data)
{
	byte request[8];
	*(uint16*)&request[0] = htons(0x1234); 
	*(uint16*)&request[2] = htons(command); 
	*(uint32*)&request[4] = htonl(data); 
	send(handle_, (const char *)request, 8, 0);
	usleep(5 * 1000); // Wait a little just to make sure that the command has been processed by Ethernet DAQ

	// // for debugging only
    // printf("Sending command: ");
	// for (int i = 0; i < 8; ++i)
	// {
	// 	printf("%02hhX", request[i]);
	// }
	// printf("\n");

}