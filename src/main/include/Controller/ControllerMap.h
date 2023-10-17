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
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
    };

    //Different enum for POV actions because logic is different
    enum POVAction{
        NO_POV_ACTION = -1,
        I_PUT_A_NEW_FORGIS_ON_THE_JEEP,
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
        {{LJOY, TRIGGER},       ZERO_DRIVE_PID},
        {{LJOY, B_4},           INTAKE},
        {{RJOY, B_3},           OUTTAKE},

        {{RJOY, X_AXIS},        SWERVE_ROTATION},
        {{RJOY, Y_AXIS},        NONE},
        {{RJOY, TRIGGER},       ZERO_YAW},
        
        {XBOX_LJOY_X,           NONE},
        {XBOX_LJOY_Y,           NONE}, 
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,           NONE},
        {XBOX_A_BUTTON ,        STOW},
        {XBOX_B_BUTTON ,        SCORE_MID},
        {XBOX_X_BUTTON ,        SCORE_LOW},
        {XBOX_Y_BUTTON ,        SCORE_HIGH},
        {XBOX_L_BUMPER ,        CONE},
        {XBOX_LTRIGGER ,        NONE},
        {XBOX_R_BUMPER ,        HP},
        {XBOX_RTRIGGER ,        GROUND}
    };

    //Allows for maps of buttons to values, such as the index of the buttonboard
    //Only for buttons and triggers currently
    //No need for default val because it's now in the controller method
    template <typename T>
    struct ValueMapElement{
        Button button;
        T value;
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
    };
};