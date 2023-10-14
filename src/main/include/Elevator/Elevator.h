//
// Created by Vir Shah on 6/14/23.
//
#pragma once

#include <ctre/Phoenix.h>
#include "ElevatorConstants.h"
#include "FeedforwardPID.h"

#include "Util/Mechanism.h"

#include "frc/DigitalInput.h"

class Elevator : public Mechanism{
    public:
        // constructor
        Elevator(bool enabled, bool shuffleboard);

        // possible states that the elevator can be in
        enum ElevatorState {
            MANUAL,
            MOVING,
            STOPPED,
        };

        enum ElevatorTarget{
            CUSTOM,
            LOW,
            MID,
            HIGH,
            STOWED
        };

        void Stow();
        void ExtendMid();
        void ExtendLow();
        void ExtendHigh();
        //extended position in meters (how far the elevator has extended from the point where it )
        void ExtendToCustomPos(double newPos);
        double GetPos();
        double GetVel();

        // debug getters
        double getElevatorHeight();
        ElevatorState getState();
        std::string getStateString();
        std::string getTargetString();

        void zero_motors();

        void setDebugManualVolts(double range);

        void activateManualMode();
        void activateMovingMode();

    private:
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;
        void CoreShuffleboardUpdate() override;

        // motors
        WPI_TalonFX left_;
        WPI_TalonFX right_;

        // limit switch
        frc::DigitalInput limit_switch_;

        // member variables
        Poses::Pose1D current_pose_;
        ElevatorState current_state_;
        ElevatorTarget current_target_;
        FeedforwardPID feedforward_;
        double max_volts_;
        double debug_manual_volts_;

        double talonUnitsToMeters(double motor_units);
        double talonUnitsToAngle(double motor_units);
};