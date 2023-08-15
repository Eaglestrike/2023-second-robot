#pragma once
namespace ClimbConstants {
        using Acceleration = units::compound_unit<units::radians_per_second,
                                            units::inverse<units::second>>;
        using kv_unit = units::compound_unit<units::volts, units::inverse<units::radians_per_second>>;
        using ka_unit = units::compound_unit<units::volts, units::inverse<Acceleration>>;
        
        const int MOTOR_ID = 0;

        //2 controllers, 1 for extending and stowing, 1 for the climbs
        const double CLIMB_MAX_VOLTAGE = 0.0;
        const double EXTND_STOW_MAX_VOLTAGE = 0.0;

        //this boolean controls whether climb is PID or FF 
        //doesn't effect extending and stowing of climb
        const bool FEEDFORWARD = true;

        // feedforward consts
        // ff is rad, rad/sec, rad/sec^2 -> volts
        const double FF_S = 0.0; // volts
        const double FF_G = 0.0; // volts
        const double FF_V = 0.0; // volts*seconds/rad
        const double FF_A = 0.0; // volts*seconds^2/rad

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