/**
 * A file of general constants
 */

#pragma once

namespace InputConstants{
    const int LJOY_PORT = 0;

    const int RJOY_PORT = 1;

    const int XBOX_PORT = 2;
    const int XBOX_LJOY_X = 0;
    const int XBOX_LJOY_Y = 1;
    const int XBOX_LTRIGGER = 2;
    const int XBOX_RTRIGGER = 3;
    const int XBOX_RJOY_X = 4;
    const int XBOX_RJOY_Y = 5;
    const int XBOX_A_BUTTON = 1;
    const int XBOX_B_BUTTON = 2;
    const int XBOX_X_BUTTON = 3;
    const int XBOX_Y_BUTTON = 4;

    const int BUTTON_BOARD_PORT = 3;
}

namespace GeneralConstants{
    //input from xbox controller
    const int STOW_BUTTON = InputConstants::XBOX_B_BUTTON;
    const int EXTEND_BUTTON = InputConstants::XBOX_A_BUTTON;
    const int LIFT_BUTTON = InputConstants::XBOX_X_BUTTON;
}