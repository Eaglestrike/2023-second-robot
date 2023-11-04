#pragma once

#include "Drive/AutoLineup.h"
#include "Elevator/ElevatorIntake.h"

class SadAuto {
public:
    enum State {
        NOT_STARTED,
        PLACING_UP,
        PLACING_ROLLERS,
        PLACING_DOWN,
        MOVING,
        DONE
    };

    SadAuto();

    void Start();
    void Periodic();
    void UpdateOdom(vec::Vector2D pos, double ang, vec::Vector2D wheelVel);

    State GetCurState();
    vec::Vector2D GetVelocity();

private:
    State m_state;

    AutoLineup &m_autoLineup;
    ElevatorIntake &m_ei;
};