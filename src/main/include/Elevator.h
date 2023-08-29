//
// Created by Vir Shah on 6/14/23.
//

#include <ctre/Phoenix.h>
#include "Constants.h"
#include "Feedforward.h"

class Elevator {
    public:
        // functions
        Elevator(int leftID, int rightID);

        // possible states that the elevator can be in
        enum ELEVATOR_STATE {
            MOVING_TO_DOCKED,
            MOVING_TO_RAISED,
            DOCKED,
            RAISED,
            STOPPED, // setState must be called to escape this state.
        }

        struct ElevatorPose {
            double velocity;
            double acceleration;
            double distance;
        }


        // debug getters
        double getElevatorHeight();
        double getLeftRotation();
        double getRightRotation();

        // util methods
        void periodic();
        void setState(Elevator::ELEVATOR_STATE new_pos);

        void zero_motors();
        void stop();
    private:
        // member variables
        Elevator::ELEVATOR_STATE current_state;
        Feedforward feedforward_;

        // motors
        WPI_TalonFX left_;
        WPI_TalonFX right_;
};