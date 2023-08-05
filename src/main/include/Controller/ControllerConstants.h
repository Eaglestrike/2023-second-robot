#pragma once

namespace ControllerConstants{
    enum Joysticks{
        NO_JOYSTICK = -1,
        LJOY,
        RJOY,
        XBOX,
        BUTTONBOARD
    };
    const int NUM_JOYSTICKS = 2;
    const int JOYSTICK_PORTS[] = {0, 1, 2, 3};
    const double DEFAULT_DEAD = 0.05;

    //Holds information of what to return: double or bool and what function to call
    enum ButtonType{
        NO_BUTTON = -1,
        AXIS_BUTTON, //Axis, which returns a double not boolean
        BUTTON_BUTTON, //Normal "Push down to activate" button
        TRIGGER_BUTTON, //Trigger on a joystick
        POV_BUTTON //TODO: NOT IMPLEMENTED
    };

    struct ButtonData{
        ButtonType type;
        int id;
    };
    //Button data for generic joystics
    const ButtonData TRIGGER = {ButtonType::TRIGGER_BUTTON, -1};
    const ButtonData X_AXIS = {ButtonType::AXIS_BUTTON, 0};
    const ButtonData Y_AXIS = {ButtonType::AXIS_BUTTON, 1};

    //Default is None
    struct Button{
        Joysticks joystick = NO_JOYSTICK;
        ButtonData data = {NO_BUTTON, -1};
    };

    const Button XBOX_LJOY_X     = {XBOX, {AXIS_BUTTON,   0}};
    const Button XBOX_LJOY_Y     = {XBOX, {AXIS_BUTTON,   1}};
    const Button XBOX_LTRIGGER   = {XBOX, {AXIS_BUTTON,   2}};
    const Button XBOX_RTRIGGER   = {XBOX, {AXIS_BUTTON,   3}};
    const Button XBOX_RJOY_X     = {XBOX, {AXIS_BUTTON,   4}};
    const Button XBOX_RJOY_Y     = {XBOX, {AXIS_BUTTON,   5}};
    const Button XBOX_A_BUTTON   = {XBOX, {BUTTON_BUTTON, 1}};
    const Button XBOX_B_BUTTON   = {XBOX, {BUTTON_BUTTON, 2}};
    const Button XBOX_X_BUTTON   = {XBOX, {BUTTON_BUTTON, 3}};
    const Button XBOX_Y_BUTTON   = {XBOX, {BUTTON_BUTTON, 4}};
    const Button XBOX_L_BUMPER   = {XBOX, {BUTTON_BUTTON, 5}};
    const Button XBOX_R_BUMPER   = {XBOX, {BUTTON_BUTTON, 6}};
}