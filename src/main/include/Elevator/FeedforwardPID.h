/**
 * @file Feedforward.h
 * @author Vir Shah (vir.shah@team114.org)
 * @brief Module for feedforward functions, coupled with PID calculator.
 * @version 0.1
 * @date 2023-08-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <frc/Timer.h>

#include "ElevatorConstants.h"

class FeedforwardPID
{
public:
    // constructor
    FeedforwardPID(ElevatorConstants::FeedforwardConfig constants, bool shuffleboard = false);

    // main methods to use
    double periodic(Poses::Pose1D current_pose);
    Poses::Pose1D getExpectedPose(double time);

    void start();
    void reset();
    void stop();

    // getters and setters
    double getKs();
    double getKv();
    double getKa();
    double getKg();

    double getKp();
    double getKd();

    double getMaxVelocity();
    double getMaxAcceleration();

    bool getReversed();

    void setKs(double ks);
    void setKv(double kv);
    void setKa(double ka);
    void setKg(double kg);
    void setMaxDistance(double distance);
    void setReversed(bool reversed);

    void setPIDConstants(double kp, double kd);

    void setMaxVelocity(double max_vel);
    void setMaxAcceleration(double max_acc);

private:
    // member functions
    double sign(double value);
    double calculateFeedforwardVoltage(double velocity, double acceleration);
    double calculatePIDVoltage(Poses::Pose1D expected, Poses::Pose1D current);
    void recalculateTimes();

    // total distance needed to travel
    double max_distance_;

    // feedforward constants
    double ks, kv, ka, kg;

    // velocity, acceleration constants
    double max_velocity;
    double max_acceleration;

    // pid constants, initialized to 0 so their use is optional
    double kp;
    double kd;

    // calculated constants
    double acceleration_time;
    double velocity_time;

    // used to control the timer
    bool isRunning;

    // to control direction of feedforward path
    bool reversed;

    // timer
    frc::Timer timer{};

    bool shuffleboard;
};