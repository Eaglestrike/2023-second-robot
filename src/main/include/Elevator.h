//
// Created by Vir Shah on 6/14/23.
//

#include <ctre/Phoenix.h>
#include "Constants.h"

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

        void zero();
        void stop();
    private:
        // member variables
        Elevator::ELEVATOR_STATE current_state;

        double left_motor_rotation = 0.0;
        double right_motor_rotation = 0.0;
        double elevator_height = 0.0;

        // feedforward constants
        double ks, kv, ka, kg;

        // pid constants
        double kp, kd;

        // motors
        WPI_TalonFX left_;
        WPI_TalonFX right_;

        // functions
        double calculateFeedforward(double velocity, double acc);
        ElevatorPose getExpectedPose();
        double elevatorPID();
};