/**
 * A file of constants
 *
 * @note General constants are not namespaced
 */

#pragma once
#include <cstddef>

const double NEAR_ZERO_TOLERANCE = 0.000001;

namespace SwerveConstants {
    const double MAG_ENCODER_COUNTS_PER_REV = 4096;
    const double TALON_FX_COUNTS_PER_REV = 2048;
    const double WHEEL_RADIUS = 0.0508;  // in meters
    const double WHEEL_GEAR_RATIO =
        6.12;  // stolen from Alex, 6.12 motor spins = 1 wheel spin

    // meters
    const double CENTER_TO_EDGE = 0.368;

    const std::size_t FR_DRIVE_ID = 4;
    const std::size_t BR_DRIVE_ID = 1;
    const std::size_t FL_DRIVE_ID = 23;
    const std::size_t BL_DRIVE_ID = 22;

    const std::size_t FR_TURN_ID = 10;
    const std::size_t BR_TURN_ID = 7;
    const std::size_t FL_TURN_ID = 5;
    const std::size_t BL_TURN_ID = 19;

    const std::size_t FR_ENCODER_ID = 8;
    const std::size_t BR_ENCODER_ID = 9;
    const std::size_t FL_ENCODER_ID = 2;
    const std::size_t BL_ENCODER_ID = 6;

    const bool FR_INVERTED = 0;
    const bool BR_INVERTED = 1;
    const bool FL_INVERTED = 0;
    const bool BL_INVERTED = 1;

    // degrees
    const double FR_OFFSET = 12.5;
    const double BR_OFFSET = -148.7;
    const double FL_OFFSET = 7.29;
    const double BL_OFFSET = -83.7;

    const double TURN_P = 3.5;
    const double TURN_I = 0;
    const double TURN_D = 0;

    const double ANG_CORRECT_P = 20;  // ±20 is good number, if you find yourself
                                    // changing this you're brwoning
    const double ANG_CORRECT_I = 0;
    const double ANG_CORRECT_D = 0.1;

    const double MAX_VOLTS = 12.0;
}  // namespace SwerveConstants

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

    const int CLIMB_STOW = XBOX_B_BUTTON;
    const int CLIMB_EXTEND = XBOX_A_BUTTON;
    const int CLIMB_LIFT = XBOX_X_BUTTON;


    const int BUTTON_BOARD_PORT = 3;
}

namespace OdometryConstants {
    const double P_INITIAL = 1.0;
    const double POS_STD_DEV = 0.1;
    const double MEASURE_STD_DEV = 0.1;
    const double CAMERA_TRUST_K = -10.0;
}  // namespace OdometryConstants

namespace ClimbConstants {
    const std::size_t MOTOR_ID = 0;

    //2 controllers, 1 for extending and stowing, 1 for the climbs
    const double CLIMB_MAX_VOLTAGE = 0.0;
    const double EXTND_STOW_MAX_VOLTAGE = 0.0;

    // pid is rad -> volts
    const double CLIMB_P = 0.0;
    const double CLIMB_I = 0.0;
    const double CLIMB_D = 0.0;
    const double EXTND_STOW_P = 0.0;
    const double EXTND_STOW_I = 0.0;
    const double EXTND_STOW_D = 0.0;

    //rad
    const double CLIMB_POS_ERR_TOLERANCE = 0.0;    
    const double EXTND_STOW_POS_ERR_TOLERANCE = 0.0;

    //rad per sec
    const double CLIMB_VEL_ERR_TOLERANCE = 0.0;   
    const double EXTND_STOW_VEL_ERR_TOLERANCE = 0.0; 

    //target positions in radians
    const double STOWED_POS = 0.0; 
    const double EXTENDED_POS = 0.0;
    const double LIFTED_POS = 0.0; 

    //input from xbox controller
    const int STOW_BUTTON = InputConstants::XBOX_B_BUTTON;
    const int EXTEND_BUTTON = InputConstants::XBOX_A_BUTTON;
    const int LIFT_BUTTON = InputConstants::XBOX_X_BUTTON;

}  // namespace ClimbConstants