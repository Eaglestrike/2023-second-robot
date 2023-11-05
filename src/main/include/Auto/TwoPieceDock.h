#pragma once

#include "Auto/BaseAuto.h"

class TwoPieceDock : public BaseAuto {
    const double ROLLER_OUTTAKE_TIME = 0.5;
    const double SPLINE_TIME_OFFSET = 0.3;
    const double INTAKE_CUBE_TIME = 1;

public:
    enum State {
        NOT_STARTED,
        PLACE1_UP,
        PLACE1_HOLD,
        PLACE1_DOWN,
        GO_TO_PIECE_2,
        INTAKE2,
        GO_TO_GRID_2,
        PLACE2_UP,
        PLACE2_HOLD,
        PLACE2_DOWN,
        GO_TO_PIECE_3,
        INTAKE3,
        GO_TO_DOCK,
        CAN_DOCK
    };

    using BaseAuto::BaseAuto;

    void Init() override;
    void Periodic() override;

    vec::Vector2D GetDriveVel() override;
    double GetAngVel() override;
    bool DockNow() override;

private:
    State m_state;

    double m_startTime;
};