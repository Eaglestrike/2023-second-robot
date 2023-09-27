#pragma once
namespace ClimbConstants {
        using Acceleration = units::compound_unit<units::radians_per_second,
                                            units::inverse<units::second>>;
        using kv_unit = units::compound_unit<units::volts, units::inverse<units::radians_per_second>>;
        using ka_unit = units::compound_unit<units::volts, units::inverse<Acceleration>>;
        
        const int MOTOR_ID = 2;

        //2 controllers, 1 for extending and stowing, 1 for the climbs
        const double CLIMB_MAX_VOLTAGE = 2.0;
        const double EXTND_STOW_MAX_VOLTAGE = 0.0;

        //this boolean controls whether climb is PID or ff-pid 
        //doesn't effect extending and stowing of climb
        const bool FEEDFORWARD = true;

        //enable or disable smart dashboard prints
        const bool SMART_DASH = true;

        // feedforward consts
        // ff is rad, rad/sec, rad/sec^2 -> volts
        const double FF_S = 0.65; // smallest amount of volts to get a non-neg velocity
        const double FF_G = 1.8; // last 2.0 //og 1.7// volts needed to keep bar hroizontal
        const double FF_V = 1/1321; // volts*seconds/rad
        // inverse of slope
        const double FF_A = 0.00001; // volts*seconds^2/rad
        //small value

        const double MAX_ACC = 0.1; // rad/sec^2
        const double MAX_VEL = 1; // rad/sec

        // pid is rad -> volts
        const double CLIMB_P = 0.0;
        const double CLIMB_I = 0.0; // not used for ff-pid
        const double CLIMB_D = 0.0;
        const double EXTND_STOW_P = 0.0;
        const double EXTND_STOW_I = 0.0;
        const double EXTND_STOW_D = 0.0;

        //rad
        const double CLIMB_POS_ERR_TOLERANCE = 0.1;  
        const double EXTND_STOW_POS_ERR_TOLERANCE = 0.1;

        //rad per sec
        const double CLIMB_VEL_ERR_TOLERANCE = 0.1;   
        const double EXTND_STOW_VEL_ERR_TOLERANCE = 0.1; 

        //target positions in radians
        const double STOWED_POS = 0.0; 
        const double EXTENDED_POS = 1.65;
        const double LIFTED_POS = -2.5;
}  // namespace ClimbConstants