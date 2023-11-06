#pragma once

#include "Drive/AutoLineup.h"
#include "Elevator/ElevatorIntake.h"
#include "Auto/BaseAuto.h"
#include <string>
#include <frc/Timer.h>
#include <iostream>

#include "Elevator/Intake/Rollers.h"


class SadAuto{
public:
    enum State {
        NOT_STARTED,
        PLACING_UP,
        PLACING_ROLLERS,
        STOWING,
        MOVING,
        DONE
    };

    SadAuto(ElevatorIntake& elevator_intake, Rollers& roller);

    void Start();
    void Periodic();
    void UpdateOdom(vec::Vector2D pos, double ang, vec::Vector2D wheelVel);

    State GetCurState();
    vec::Vector2D GetVelocity();

    void debugChangeTime(double new_time);

private:
    State m_state;

    frc::Timer timer{};
    ElevatorIntake &m_ei;
    Rollers &m_rollers;

    svector::Vector2D target{0.5, 0.0};
    
    std::string StateToString(State state);

    double time_with_velocity = 1.0;
    double time_outtaking = 2.0;
    svector::Vector2D m_vel{0.0, 0.0};
};