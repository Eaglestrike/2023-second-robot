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

        void setFeedforwardConstants(double ks, double kv, double kg, double ka);
        void setPIDConstants(double kp, double kd);

        // util methods
        void periodic();
        void setState(ElevatorState new_pos);

        void zero_motors();
        void stop();
    private:
        // member variables
        ElevatorState current_state;
        Feedforward feedforward_;

        // motors
        WPI_TalonFX left_;
        WPI_TalonFX right_;
};