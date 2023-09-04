#pragma once

#include <frc/SerialPort.h>
#include <frc/Timer.h>

namespace LidarReaderConstants{
    const char REQ[] = {0x59}; //Request write char
    const char RES = 0x59; //Respond read char

    const int BAUD_RATE = 115200;
    const frc::SerialPort::Port LIDAR_PORT = frc::SerialPort::kMXP;
    const double RESPONSE_TIME = 1.0;

    const char NO_READ = 255; //Value if lidar doesn't see anything

    const int READ_BUFFER_SIZE = 8;

    const double DEFAULT_POSITION = 30.0; //cm
};

class LidarReader{
    public:
        struct LidarData{
            double conePos;
            double cubePos;
            bool hasCone;
            bool hasCube;
            bool isValid;
        };

        LidarReader();
        void RequestData();
        void Periodic();

        LidarData getData(){return data_;}
        double getConePos(){return data_.conePos;}
        double getCubePos(){return data_.cubePos;}
        bool hasCone(){return data_.hasCone;}
        bool hasCube(){return data_.hasCube;}
        bool validData(){return data_.isValid;}

    private:
        bool checkValid(const char data[4]);
        void findOffset();

        frc::SerialPort port_;
        double reqTime_; //Time since last request
        bool isRequesting_ = false; //If currently there is a call

        char readBuffer_[12]; //Reads max 12 bytes
        char readData_[7]; //[3 old values (for checks/adjustments), RES, cone, cube, check]
        int readIndex_ = 0; //Number of bytes currently read

        LidarData data_;
};