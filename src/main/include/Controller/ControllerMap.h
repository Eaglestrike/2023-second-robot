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
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
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

        {{RJOY, X_AXIS},        SWERVE_ROTATION},
        {{RJOY, Y_AXIS},        NONE},
        {{RJOY, TRIGGER},       ZERO_YAW},
        
        {XBOX_LJOY_X,           NONE},
        {XBOX_LJOY_Y,           NONE}, 
        {XBOX_LTRIGGER,         NONE},
        {XBOX_RTRIGGER,         NONE},
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,           NONE},
        {XBOX_A_BUTTON ,        NONE},
        {XBOX_B_BUTTON ,        NONE},
        {XBOX_X_BUTTON ,        NONE},
        {XBOX_Y_BUTTON ,        NONE},
        {XBOX_L_BUMPER ,        NONE},
        {XBOX_R_BUMPER ,        NONE}
    };
};