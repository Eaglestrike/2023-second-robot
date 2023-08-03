#pragma once

namespace ClimbConstants {
    const int MOTOR_ID = 0;

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
}  // namespace ClimbConstants