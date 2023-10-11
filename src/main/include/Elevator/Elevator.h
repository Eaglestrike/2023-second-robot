//
// Created by Vir Shah on 6/14/23.
//

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
            MOVING
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
        void CoreTeleopPeriodic() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;
        void CoreShuffleboardUpdate() override;
        // member variables
        ElevatorState current_state_;
        FeedforwardPID feedforward_;

        // motors
        WPI_TalonFX left_;
        WPI_TalonFX right_;

        double talonUnitsToMeters(double motor_units);
        double talonUnitsToAngle(double motor_units);
        void evaluateState();
        void evaluateDirection();
};