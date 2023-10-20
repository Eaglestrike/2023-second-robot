#include "Elevator/Lidar/LidarReader.h"

#include <iostream>
#include <algorithm>

#include "frc/smartdashboard/SmartDashboard.h"

/// @brief Constructor
LidarReader::LidarReader(bool enabled, bool shuffleboard):
    Mechanism("Lidar", enabled, shuffleboard),
    port_(LidarReaderConstants::BAUD_RATE, LidarReaderConstants::LIDAR_PORT),
    isRequesting_(false)
{
    frc::SmartDashboard::PutBoolean("Lidar Stale", true);
    double time = frc::Timer::GetFPGATimestamp().value();
    reqTime_ = time;
    data_ = LidarData{
            .conePos = LidarReaderConstants::DEFAULT_POSITION,
            .cubePos = LidarReaderConstants::DEFAULT_POSITION,
            .hasCone = false,
            .hasCube = false,
            .isValid = false,
            .readTime = time
        };
    port_.SetTimeout(1_s);
    port_.EnableTermination('\n');
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
void LidarReader::CorePeriodic(){
    double time = frc::Timer::GetFPGATimestamp().value();
    
    frc::SmartDashboard::PutBoolean("Lidar Stale", !data_.isValid);
    frc::SmartDashboard::PutNumber("Port read size", port_.GetBytesReceived());

    if(autoRequest_){
        RequestData();
    }

    //Check how stale data is
    if(time - data_.readTime > LidarReaderConstants::VALID_DATA_TIME){
        data_.isValid = false;
    }

    //If not requesting, do nothing
    if(!isRequesting_){
        return;
    }

    //Check if it is responding in time
    if(time - reqTime_ > LidarReaderConstants::RESPONSE_TIME){
        isRequesting_ = false;
        port_.Reset();
        std::cout<<"Failed Requesting Lidar Data"<<std::endl;
    }

    readData();
}

/// @brief Sets the autorequest config
/// @param autoRequest requests data if invalid data
void LidarReader::setAutoRequest(bool autoRequest){
    autoRequest_ = autoRequest;
}

/// @brief Reads the data from the port
void LidarReader::readData(){
    //Check buffer size
    int bufferSize = port_.GetBytesReceived();
    if(bufferSize == 0){
        return;
    }
    
    //Read Data
    if(bufferSize > 1000){ //If disconnected, port gets filled
        std::cout<<"Reset Port"<<std::endl;
        port_.Reset();
        return;
    }

    while (bufferSize > 8){ //Clear port
        int cleared = port_.Read(clearBuffer_, std::min(1024, bufferSize - 8));
        bufferSize -= cleared;
    }

    int count = port_.Read(readBuffer_, 8);

    //Verify and store data
    for(int i = 0; i < count; i++){
        readData_[readIndex_ + 4] = readBuffer_[i]; //Stores in last 4 bytes of array
        readIndex_++;
        if(readIndex_ == 4){ //Has read 4 bytes
            readIndex_ = 0;
            isRequesting_ = false;

            //Check recorded data is good
            if(isValidData(readData_ + 4)){
                storeData(readData_ + 4);
            }
            else{
                findOffset();
            }

            //Shifts the array by 4 to left
            std::copy(readData_+4, readData_+8, readData_);
        }
    }
}

/// @brief stores the data
void LidarReader::storeData(const unsigned char data[4]){
    data_.hasCone = (readData_[1] != LidarReaderConstants::NO_READ);
    data_.hasCube = (readData_[2] != LidarReaderConstants::NO_READ);
    data_.conePos = data_.hasCone? ((double)readData_[1]) : LidarReaderConstants::DEFAULT_POSITION;
    data_.cubePos = data_.hasCube? ((double)readData_[2]) : LidarReaderConstants::DEFAULT_POSITION;
    data_.isValid = true;
    data_.readTime = frc::Timer::GetFPGATimestamp().value();
}

/// @brief Checks if data is valid via the check sum and response key
/// @param data 4 chars
/// @return boolean
bool LidarReader::isValidData(const unsigned char data[4]){
    //Respond invalid
    if(data[0] != LidarReaderConstants::RES){
        return false;
    }
    //Check sum
    if(((data[0] + data[1] + data[2]) % 256) != data[3]){
        return false;
    }
    return true;
}

/// @brief finds the offset, then changes the readindex
void LidarReader::findOffset(){
    //Check all possible offsets
    for(int off = 1; off < 4; off++){
        if(isValidData(readData_ + off)){
            storeData(readData_ + off);
            //Shift data left
            std::copy(readData_ + off, readData_ + 8, readData_);
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

/// @brief Returns when the data was recorded
/// @return seconds
double LidarReader::getRecordedTime(){
    return data_.readTime;
}

void LidarReader::CoreShuffleboardInit(){
    frc::SmartDashboard::PutBoolean("Get Data", false);
    frc::SmartDashboard::PutBoolean("Auto Request", false);
    frc::SmartDashboard::PutNumber("Cone Pos", getConePos());
    frc::SmartDashboard::PutNumber("Cube Pos", getCubePos());
    frc::SmartDashboard::PutBoolean("Has Cube", hasCube());
    frc::SmartDashboard::PutBoolean("Has Cone", hasCone());
    frc::SmartDashboard::PutBoolean("Is Requesting", isRequesting_);
    frc::SmartDashboard::PutNumber("Read Time", data_.readTime);
}

void LidarReader::CoreShuffleboardPeriodic(){
    setAutoRequest(frc::SmartDashboard::GetBoolean("Auto Request", false));
    if(frc::SmartDashboard::GetBoolean("Get Data", false)){
        RequestData();
        frc::SmartDashboard::PutBoolean("Get Data", false);
    }
    frc::SmartDashboard::PutNumber("Cone Pos", getConePos());
    frc::SmartDashboard::PutNumber("Cube Pos", getCubePos());
    frc::SmartDashboard::PutBoolean("Has Cube", hasCube());
    frc::SmartDashboard::PutBoolean("Has Cone", hasCone());
    frc::SmartDashboard::PutBoolean("Is Requesting", isRequesting_);
    frc::SmartDashboard::PutNumber("Read Time", data_.readTime);
}