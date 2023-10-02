#include "Elevator/Lidar/LidarReader.h"

#include <iostream>
#include <algorithm>

#include "frc/smartdashboard/SmartDashboard.h"

/// @brief Constructor
LidarReader::LidarReader():
    port_(LidarReaderConstants::BAUD_RATE, LidarReaderConstants::LIDAR_PORT)
{
    frc::SmartDashboard::PutBoolean("Lidar Responding", true);
}

/// @brief Requests Lidar for data
void LidarReader::RequestData(){
    if(isRequesting_){
        return;
    }
    port_.Write(LidarReaderConstants::REQ, 1);
    reqTime_ = frc::Timer::GetFPGATimestamp().value();
    isRequesting_ = true;
}

/// @brief Should be called in perioidic - reads port and stores data
void LidarReader::Periodic(){
    //If not requesting, do nothing
    if(!isRequesting_){
        return;
    }
    //Futher logic assumes a request is in place

    //Check if it is responding in time
    if(frc::Timer::GetFPGATimestamp().value() - reqTime_ > LidarReaderConstants::RESPONSE_TIME){
        data_.isValid = false;
        isRequesting_ = false;
        port_.Reset();
        RequestData();
    }
    else{
        data_.isValid = true;
    }
    frc::SmartDashboard::PutBoolean("Lidar Responding", data_.isValid);

    //Check buffer size
    int bufferSize = port_.GetBytesReceived();
    if(bufferSize == 0){
        return;
    }
    
    //Read Data
    if(bufferSize > 8){
        std::cout<<"Big Lidar Buffer"<<std::endl;
        for(int s = 0; s < bufferSize - 8; s += 4){ //Clear Buffer in intervals of 
            port_.Read(readBuffer_, 4);
        }
    }
    int count = port_.Read(readBuffer_, 8);

    //Verify and store data
    for(int i = 0; i < count; i++){
        readData_[readIndex_ + 2] = readBuffer_[i];
        readIndex_++;
        //Has read 4 bytes
        if(readIndex_ == 4){
            //Reset reading to start of data
            readIndex_ = 0;
            isRequesting_ = false;

            //Checks Validity of read data
            if(checkValid(readData_)){
                data_.hasCone = readData_[1] == LidarReaderConstants::NO_READ;
                data_.hasCube = readData_[2] == LidarReaderConstants::NO_READ;
            }
            else{
                data_.hasCone = false;
                data_.hasCube = false;
                findOffset();
            }
            data_.hasCone = data_.hasCone? ((double)readData_[1]) : LidarReaderConstants::DEFAULT_POSITION;
            data_.hasCube = data_.hasCube? ((double)readData_[2]) : LidarReaderConstants::DEFAULT_POSITION;

            //Fills the last 3 bytes of lastData
            std::copy(readData_ + 4, readData_ + 7, readData_);
        }
    }
}

/// @brief Checks if data is valid via the check sum
/// @param data 4 chars
/// @return boolean
bool LidarReader::checkValid(const char data[4]){
    //Respond invalid
    if(data[0] != LidarReaderConstants::RES){
        return false;
    }
    //Check sum
    if((data[0] + data[1] + data[2]) % 256 != data[4]){
        return false;
    }
    return true;
}

/// @brief finds the offset, then changes the readindex
void LidarReader::findOffset(){
    for(int off = 0; off<3; off++){
        //Check offset
        if(checkValid(readData_ + off)){
            //Shift data
            std::copy(readData_ + off + 1, readData_ + 7, readData_);
            readIndex_ = off+1;
            return;
        }
    }
}

/// @brief Gets the current data
/// @return LidarData struct with all info
LidarReader::LidarData LidarReader::getData(){
    return data_;
}

/// @brief Gets the cone position in cm
/// @return cm
double LidarReader::getConePos(){
    return data_.conePos;
}

/// @brief Gets the cube position in cm
/// @return cm
double LidarReader::getCubePos(){
    return data_.cubePos;
}

/// @brief returns if the lidar senses a cone
/// @return if the lidar senses a cone
bool LidarReader::hasCone(){
    return data_.hasCone;
}

/// @brief returns if the lidar senses a cube
/// @return if the lidar senses a cube
bool LidarReader::hasCube(){
    return data_.hasCube;
}

/// @brief returns if the data is valid (collected within some time)
/// @return if data is valid
bool LidarReader::validData(){
    return data_.isValid;
}
