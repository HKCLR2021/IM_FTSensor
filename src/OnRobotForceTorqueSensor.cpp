#include "ft_sensor/OnRobotForceTorqueSensor.hpp"

std::map<std::string, std::weak_ptr<OnRobotForceTorqueSensor>> OnRobotForceTorqueSensor::instances;

std::shared_ptr<OnRobotForceTorqueSensor> OnRobotForceTorqueSensor::getInstance(
    std::string ipAddress,
    bool fake_mode,
    int32 sampleingHz,
    int32 filterType,
    bool enableBiasing
){
    auto search = instances.find(ipAddress);
    printf("Find ft sensor at %s\n", search->first.c_str());
    if (search != instances.end()){
        auto instance = search->second;
        if (!instance.expired()) {
            printf("Return existing instance of OnRobot FT Sensor at %s, other config params passed are ignored\n", ipAddress.c_str());
            return instance.lock();
        }
    }

    auto instance = std::shared_ptr<OnRobotForceTorqueSensor>(
        new OnRobotForceTorqueSensor(ipAddress, sampleingHz, filterType, enableBiasing, fake_mode)
    );
    instances[ipAddress] = instance;
    return instance;
}

OnRobotForceTorqueSensor::OnRobotForceTorqueSensor(std::string ipAddress, int32 sampleingHz, int32 filterType, bool enableBiasing, bool fake_mode)
{
    fake_mode_ = fake_mode;
    if(!isFake())
    {
        handle_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (handle_ == -1) {
            fprintf(stderr, "Error, Socket could not be opened.\n");
        }

        // Connect sensor
        this->ipAddress = ipAddress;
        openDevice(ipAddress.c_str(), PORT);

        printf("OnRobot FT Sensor at %s connected, Initializing...\n", ipAddress.c_str());

        // Init with default settings
        setSamplingRate(sampleingHz);    // 100Hz
        setFilterType(filterType);       // default: 4 i.e 15Hz LPF
        setEnableBiasing(enableBiasing); // biasing on
    }

    printf("OnRobot Initialization done.\n");
}

OnRobotForceTorqueSensor::~OnRobotForceTorqueSensor(){
    stopStreaming();
    usleep(samplingDt_us*(SAMPLE_COUNT+2)); // wait till rx_thread is done (?)
    if(!isFake())
        closeDevice();
    printf("OnRobot FT Sensor at %s closed\n", ipAddress.c_str());
}


int OnRobotForceTorqueSensor::openDevice(const char * ipAddress, uint16 port)
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

Response OnRobotForceTorqueSensor::receive()
{
    byte inBuffer[36];
	Response response;
	unsigned int uItems = 0;
	int status = recv(handle_, (char *)inBuffer, 36, 0 );
    if (status<0) return response;

    if (!isFake())
    {
        response.sequenceNumber = ntohl(*(uint32*)&inBuffer[0]);
        response.sampleCounter = ntohl(*(uint32*)&inBuffer[4]);
        response.status = ntohl(*(uint32*)&inBuffer[8]);
        response.fx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
        response.fy = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
        response.fz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
        response.tx = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
        response.ty = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
        response.tz = (ntohl(*(int32*)&inBuffer[12 + (uItems++) * 4]));
    }
    else
    {
        response.sequenceNumber = -1.0;
        response.sampleCounter = -1.0;
        response.status = -1.0;
        response.fx = -1.0;
        response.fy = -1.0;
        response.fz = -1.0;
        response.tx = -1.0;
        response.ty = -1.0;
        response.tz = -1.0;
    }

	return response;
}

void OnRobotForceTorqueSensor::getLatestDataDArray(std::array<double, 6> &data_darr)
{
    rx_lck_.lock();
    std::copy(std::begin(data_buf_), std::end(data_buf_), std::begin(data_darr) );
    rx_lck_.unlock();
}


void OnRobotForceTorqueSensor::getLatestDataVec(std::vector<double> &data_dvec)
{
    rx_lck_.lock();
    data_dvec = std::vector<double> (std::begin(data_buf_), std::end(data_buf_) );
    rx_lck_.unlock();
}


void OnRobotForceTorqueSensor::showResponse(Response r)
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

bool OnRobotForceTorqueSensor::setSamplingRate(int32 samplingHz)
{
    this->samplingHz = samplingHz;
    this->samplingDt_ms = 1000 / samplingHz;
    this->samplingDt_us = this->samplingDt_ms * 1000;

    sendCommand(COMMAND_SPEED, this->samplingDt_ms);

    return true;
}

bool OnRobotForceTorqueSensor::setFilterType(int32 filter_type)
{
    /*  0 = No filter;
        1 = 500 Hz;
        2 = 150 Hz;
        3 = 50 Hz;
        4 = 15 Hz;
        5 = 5 Hz;
        6 = 1.5 Hz */
    sendCommand(COMMAND_FILTER, filter_type);
    return true;
}

bool OnRobotForceTorqueSensor::setEnableBiasing(bool biasing_on)
{
    uint32 cmd_bias = biasing_on ? 0xFF : 0X00;
    sendCommand(COMMAND_BIAS, cmd_bias);
    return true;
}

void* OnRobotForceTorqueSensor::static_rx_thread(void* pThis)
{
    static_cast<OnRobotForceTorqueSensor*>(pThis)->rx_thread();
}

void OnRobotForceTorqueSensor::rx_thread()
{
    if (isFake())
    {
        rx_lck_.lock();
        data_buf_ = {-1., -1., -1., -1., -1., -1.};
        rx_lck_.unlock();
        usleep(1000000);
    }
    else
    {
        for (int trial=0;trial<5;trial++){
            sendCommand(COMMAND_START, 0);

            byte inBuffer[36];
            int status = recv(handle_, (char *)inBuffer, 36, MSG_DONTWAIT );
            if (status>0) break;
            
            usleep(samplingDt_us);
        }

        while (!rx_stop_)
        {
            Response r = receive();
            if (verbose_) showResponse(r); // logging data

            // Data formatting
            double fx = r.fx / FORCE_DIV;
            double fy = r.fy / FORCE_DIV;
            double fz = r.fz / FORCE_DIV;
            double tx = r.tx / TORQUE_DIV;
            double ty = r.ty / TORQUE_DIV;
            double tz = r.tz / TORQUE_DIV;

            rx_lck_.lock();
            data_buf_ = {fx, fy, fz, tx, ty, tz};
            rx_lck_.unlock();

            usleep(samplingDt_us);
        }

        for (int trial=0;trial<3;trial++){
            sendCommand(COMMAND_STOP, 0);
        }

    }
    pthread_exit(0);

}

void OnRobotForceTorqueSensor::startStreaming(bool verbose)
{
    verbose_ = verbose;

    rx_stop_ = false;
    if (pthread_create(&threadid_, NULL, static_rx_thread, (void*)this ) != 0 )
    {
        printf("start streaming failed\n");
        pthread_detach(threadid_);
    }
}

void OnRobotForceTorqueSensor::stopStreaming()
{
    rx_stop_ = true;
}

void OnRobotForceTorqueSensor::sendCommand(uint16 command, uint32 data)
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

bool OnRobotForceTorqueSensor::isFake()
{
    return fake_mode_;
}