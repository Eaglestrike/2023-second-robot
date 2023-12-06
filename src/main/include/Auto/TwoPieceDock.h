#pragma once

#include "Auto/BaseAuto.h"
#include "RobotStuff.h"

class TwoPieceDock : public BaseAuto {
    const double ROLLER_OUTTAKE_TIME = 0.3;
    const double SPLINE_TIME_OFFSET = 0.3;
    const double INTAKE_CUBE_TIME = 0.5;

    const AutoPaths::SwervePose ORIG_PIECE_2_GND = {3, 6.5, 0.923, 0, 0, 0, 0};
    const AutoPaths::SwervePose ORIG_PIECE_2_SCORE = {3, 1.93, 1.07, 0, 0, M_PI, 0};

    const AutoPaths::SwervePose ORIG_PIECE_3_MID = {1.5, 4.525, 0.6, 1, 0, 0, 0};
    const AutoPaths::SwervePose ORIG_PIECE_3_GND = {3, 7.16, 2.18, 0, 0, 0.55, 0};

    const AutoPaths::SwervePose ORIG_PRE_DOCK = {0.7, 6, 2.13, -AutoConstants::PRE_DOCK_SPEED, 0, M_PI / 2, 0};

    // first argument is time OFFSET
    AutoPaths::SwervePose PIECE_2_GND;
    AutoPaths::SwervePose PIECE_2_SCORE;

    AutoPaths::SwervePose PIECE_3_MID;
    AutoPaths::SwervePose PIECE_3_GND;

    AutoPaths::SwervePose PRE_DOCK;

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

    double GetAbsAng();

    bool m_doOnce;
};