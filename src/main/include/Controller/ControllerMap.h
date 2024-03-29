#pragma once

#include <vector>

#include "ControllerConstants.h"

namespace Actions{
    enum Action{
        NONE = -1,
        SWERVE_STRAFEX,
        SWERVE_STRAFEY,
        SWERVE_ROTATION,
        ZERO_DRIVE_PID,
        ZERO_YAW,
        CONE,
        SCORE_LOW,
        SCORE_MID,
        SCORE_HIGH,
        STOW,
        SET_MANUAL,
        SET_MOVING,
        HP,
        GROUND,
        INTAKE,
        OUTTAKE,
        MANUAL1,
        MANUAL2,
        ELEVATOR_H,
        INTAKE_ANG,
        AUTO_LINEUP,
        SLOW,
        GROUND_INTAKE,
        LOCK_WHEELS,
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
    };

    //Different enum for POV actions because logic is different
    enum POVAction{
        NO_POV_ACTION = -1,
        INTAKE_FLANGE,
        CUBE_INTAKE,
        ACTION_COUNT_POV //Just the number of actions, as it is at the end of a enum
    };
}

namespace ControllerMapData{
    using namespace ControllerConstants;
    using namespace Actions;

    struct ControlMapElement{
        Button button;
        Action action;
    };

    //Maps Buttons -> Actions
    //Buttons are structs in the form of {Joystick, ButtonData}
    //There are already some named ButtonData and Buttons
    const std::vector<ControlMapElement> ButtonMap = {
        {{LJOY, X_AXIS},        SWERVE_STRAFEX},
        {{LJOY, Y_AXIS},        SWERVE_STRAFEY},
        {{LJOY, TRIGGER},       AUTO_LINEUP},
        {{LJOY, B_4},           OUTTAKE},
        {{LJOY, B_2},           LOCK_WHEELS},
        {{RJOY, B_3},           INTAKE},
        {{RJOY, X_AXIS},        SWERVE_ROTATION},
        {{RJOY, Y_AXIS},        NONE},
        {{RJOY, TRIGGER},       NONE},
        {{RJOY, B_2},           SLOW},

        {XBOX_LJOY_X,           NONE},
        {XBOX_LJOY_Y,           ELEVATOR_H}, 
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,           INTAKE_ANG},
        {XBOX_A_BUTTON ,        STOW},
        {XBOX_B_BUTTON ,        SCORE_MID},
        {XBOX_X_BUTTON ,        SCORE_LOW},
        {XBOX_Y_BUTTON ,        SCORE_HIGH},
        {XBOX_L_BUMPER ,        GROUND_INTAKE},
        {XBOX_LTRIGGER ,        MANUAL1},
        {XBOX_R_BUMPER ,        HP},
        {{XBOX, B_7} ,          ZERO_DRIVE_PID},
        {{XBOX, B_8} ,          ZERO_YAW},
        {XBOX_RTRIGGER ,        MANUAL2}
    };

    //Allows for maps of buttons to values, such as the index of the buttonboard
    //Only for buttons and triggers currently
    //No need for default val because it's now in the controller method
    template <typename T>
    struct ValueMapElement{
        Button button;
        T value;
    };

     const std::vector<ValueMapElement<int>> SCORING_POS = {
        {{BUTTONBOARD, B_1}, 1},
        {{BUTTONBOARD, B_2}, 2},
        {{BUTTONBOARD, B_3}, 3},
        {{BUTTONBOARD, B_4}, 4},
        {{BUTTONBOARD, B_5}, 5},
        {{BUTTONBOARD, B_6}, 6},
        {{BUTTONBOARD, B_7}, 7},
        {{BUTTONBOARD, B_8}, 8},
        {{BUTTONBOARD, B_9}, 9},
    };

     const std::vector<ValueMapElement<int>> GET_LEVEL = {
        {BB_L1, 1},
        {BB_L2, 2},
        {BB_L3, 3}
    };

    const double TRIM_SIZE = 0.05; // in m
    const std::vector<ValueMapElement<double>> GET_TRIM_X = {
        {BB_X_TRIM_UP, TRIM_SIZE},
        {BB_X_TRIM_DOWN, -TRIM_SIZE}
    };

    const std::vector<ValueMapElement<double>> GET_TRIM_Y = {
        {BB_Y_TRIM_UP, TRIM_SIZE},
        {BB_Y_TRIM_DOWN, -TRIM_SIZE}
    };

    //Takes the range from min to max
    //if range covers over 0, like from 350->10, the larger number comes first
    struct POVRange{
        int min;
        int max;
    };

    const POVRange POV_UP = {350, 10};
    const POVRange POV_RIGHT = {80, 100};
    const POVRange POV_DOWN = {170, 190};
    const POVRange POV_LEFT = {260, 280};

    struct POVMapElement{
        Button pov;
        POVRange range;
        POVAction action;
    };

    const std::vector<POVMapElement> POVMap = {
        {XBOX_POV, POV_LEFT, INTAKE_FLANGE},
    };
};