//
// Created by Vir Shah on 6/14/23.
//

#include <ctre/Phoenix.h>
#include "ElevatorConstants.h"
#include "FeedforwardPID.h"

class Elevator {
    public:
        // constructor
        Elevator();

        // possible states that the elevator can be in
        enum ElevatorState {
            MOVING_TO_DOCKED,
            MOVING_TO_RAISED,
            DOCKED,
            RAISED,
            STOPPED, // setState must be called to escape this state.
        };

        // debug getters
        double getElevatorHeight();
        double getLeftRotation();
        double getRightRotation();
        ElevatorState getState();

        void setFeedforwardConstants(double ks, double kv, double kg, double ka);
        void setPIDConstants(double kp, double kd);
        void setMaxDistance(double distance);
        void setState(ElevatorState new_state);

        // util methods
        void periodic();

        void zero_motors();
        void stop();
        void start();

    private:
        // member variables
        ElevatorState current_state;
        FeedforwardPID feedforward_;

        // motors
        WPI_TalonFX left_;
        WPI_TalonFX right_;

        double talonUnitsToMeters(double motor_units);
        double talonUnitsToAngle(double motor_units);
        void evaluateState();
        void evaluateDirection();
};