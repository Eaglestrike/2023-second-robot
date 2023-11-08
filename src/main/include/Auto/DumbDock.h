#pragma once 
#include "BaseAuto.h"

class DumbDock {
    public:
        const double OUT_SPEED = 2.0; //swerve out speed
        const double OUT_TIME = 2.25;
        const double IN_SPEED = 0.5; // not used
        const double IN_TIME = 0.0; // not used
        const double OUTTAKE_TIME = 1.0;
        const double DONE_WAIT_TIME = 1.0;

        enum State{
            NOT_STARTED,
            IN,
            SCORE,
            OUTTAKE,
            PREP4EXIT,
            EXIT_COMMUNITY,
            WAIT,
            DONE
        };

        DumbDock (ElevatorIntake& e, Rollers& r);
        void SetSide (bool blue);
        bool CanDock();
        void Periodic();
        void Start();
        void Reset();
        vec::Vector2D GetVel();
        double GetAngleVel();
    private:
        State m_state = NOT_STARTED;
        bool m_cube = true;
        bool m_blue = true;
        double m_curTime, m_startTime;
        vec::Vector2D m_vel = {0.0, 0.0};
        ElevatorIntake& m_ei;
        Rollers& m_rollers;
};