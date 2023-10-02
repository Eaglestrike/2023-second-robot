#pragma once

#include <frc/SerialPort.h>
#include <frc/Timer.h>

namespace LidarReaderConstants{
    const char REQ[] = {0x59}; //Request write char
    const char RES = 0x59; //Respond read char

    const int BAUD_RATE = 115200; //Speed of communication
    const frc::SerialPort::Port LIDAR_PORT = frc::SerialPort::kMXP;

    const double RESPONSE_TIME = 1.0; //Maximum time to respond for valid data

    const char NO_READ = 255; //Value if lidar doesn't see anything

    const double DEFAULT_POSITION = 30.0; //cm
};

class LidarReader{
    public:
        struct LidarData{
            double conePos; //cm
            double cubePos; //cm
            bool hasCone;
            bool hasCube;
            bool isValid;
        };

        LidarReader();
        void RequestData();
        void Periodic();

        LidarData getData();
        double getConePos();
        double getCubePos();
        bool hasCone();
        bool hasCube();
        bool validData();

    private:
        void storeData(const char data[4]);
        bool isValidData(const char data[4]);
        void findOffset();

        frc::SerialPort port_;
        double reqTime_; //Time since last request
        bool isRequesting_; //If currently there is a call

        char readBuffer_[8]; //Reads to this char array
        char readData_[8]; //[4 old values (for checks/adjustments), RES, cone, cube, check]
        int readIndex_ = 0; //Number of bytes currently read

        LidarData data_;
};