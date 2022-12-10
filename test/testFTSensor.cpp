#include "ft_sensor/AdmittanceController.hpp"
#include "ft_sensor/OnRobotForceTorqueSensor.hpp"
#include <stdlib.h>
#include <iostream>

template <typename T>
void printData(T& DataArray)
{
    // std::cout << "Data: "<< DataArray[2] << std::endl; 
    for (auto &data : DataArray)
    {
        std::cout << data << " ";
        std::cout << std::endl;
    }
}

// Direct force detection using raw data
void testRawDataFTDetection()
{
    printf("[1] testRawDataFTDetection\n");
    printf("Press any key to start\n");
    std::cin.get();
    
    auto ft_sensor = OnRobotForceTorqueSensor::getInstance("192.168.1.1");
    ft_sensor->startStreaming();

    printf("get double array Data\n");
    std::array<double, 6> data_darr;
    for (int i = 0; i < 100; ++i)
    {
        ft_sensor->getLatestDataDArray(data_darr);
        printData(data_darr);
        usleep(10000); // 10ms
    }

    printf("get double vector Data\n");
    std::vector<double> data_vec;
    for (int i = 0; i < 100; ++i)
    {
        ft_sensor->getLatestDataVec(data_vec);
        printData(data_vec);
        usleep(10000); // 10ms
    }

    ft_sensor->stopStreaming();
    return;
}

// Energy tanke based method using admittance controller 
void testEnergyTankFTDetection()
{
    printf("[2] testEnergyTankFTDetection\n");
    printf("Press any key to start\n");
    std::cin.get();

    AdmittanceController admt_ctrl;
    auto ft_sensor = OnRobotForceTorqueSensor::getInstance();
    ft_sensor->startStreaming();

    std::vector<double> data_vec;
    for (int i = 0; i < 3000; ++i)
    {
        ft_sensor->getLatestDataVec(data_vec);
        Eigen::VectorXd F_ext = Eigen::VectorXd::Map(data_vec.data(), data_vec.size() );
        std::cout << std::endl;
        std::cout << i << ": " << admt_ctrl.computeAdmtOutput(F_ext).transpose() << std::endl;
        std::cout << admt_ctrl.getAdmittanceRatio() << std::endl; 
        usleep(10000); // 10ms
    }

    ft_sensor->stopStreaming();
    return;
}

int main(int argc, char* argv[])
{
    printf("START FORCE TORQUE SENSOR TESTING\n");

    testRawDataFTDetection();

    testEnergyTankFTDetection();
    
    return 0;
}