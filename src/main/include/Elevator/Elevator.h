//
// Created by Vir Shah on 6/14/23.
//
#pragma once

#include <ctre/Phoenix.h>
#include "ElevatorConstants.h"
#include "FeedforwardPID.h"

#include "Util/Mechanism.h"

class Elevator : public Mechanism{
    public:
        // constructor
        Elevator(bool enabled, bool shuffleboard);

        // possible states that the elevator can be in
        enum ElevatorState {
            MANUAL,
            HOLDING_POSITION,
            MOVING,
            STOPPED,
        };

        enum ElevatorTarget{
            LOW,
            MID,
            HIGH,
            STOWED
        };

        void ExtendMid();
        void ExtendLow();
        void ExtendHigh();
        //extended position in meters (how far the elevator has extended from the point where it )
        void ExtendToCustomPos(double newPos);
        double GetPos();
        double GetVel();

        // debug getters
        double getElevatorHeight();
        double getLeftRotation();
        double getRightRotation();
        ElevatorState getState();

        void zero_motors();

    private:
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;
        void CoreShuffleboardUpdate() override;

        // motors
        WPI_TalonFX left_;
        WPI_TalonFX right_;

        // member variables
        Poses::Pose1D current_pose_;
        ElevatorState current_state_;
        ElevatorTarget current_target_;
        FeedforwardPID feedforward_;
        double max_volts_;

        double talonUnitsToMeters(double motor_units);
        double talonUnitsToAngle(double motor_units);
};