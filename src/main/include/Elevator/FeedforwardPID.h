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

#include <frc/Timer.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include "ElevatorConstants.h"

class FeedforwardPID
{
public:
    // constructor
    FeedforwardPID(double ks, double kv, double ka, double kg, double distance);
    FeedforwardPID(double ks, double kv, double ka, double kg, double kp, double kd, double distance);

    // main methods to use
    double periodic(Poses::Pose1D current_values);

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
    Poses::Pose1D getExpectedPose(double time);

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

    // total distance needed to travel
    double max_distance_;

    // feedforward constants
    double ks, kv, ka, kg;

    // pid constants, initialized to 0 so their use is optional
    double kp = 0.0;
    double kd = 0.0;

    // velocity, acceleration constants
    double max_velocity;
    double max_acceleration;

    // used to control the timer
    double isRunning = false;

    // to control direction of feedforward path
    bool reversed = false;

    // timer
    frc::Timer timer{};
};