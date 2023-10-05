#pragma once 

namespace IntakeConstants{
    const int WRIST_MOTOR_ID = 4;
    const int ROLLER_MOTOR_ID = 31;
    const int WRIST_ENCODER_CAN_ID = 0;

    //EXTEND DEPLOY FEEDFORWARD-PID

    //feedforward constants
    const double EXTEND_DEPLOY_S = 0.1; // volts
    const double EXTEND_DEPLOY_G = 0.45; // volts needed to resist gravity
    const double EXTEND_DEPLOY_V = 0.9; // volts*seconds/rad
    const double EXTEND_DEPLOY_A = 0.02; // volts*seconds^2/rad

    //pid accounts for velocity and position error
    const double EXTEND_DEPLOY_P = 1.0; // corrects position error
    const double EXTEND_DEPLOY_I = 0.0; // accounts for cumulative pos err
    const double EXTEND_DEPLOY_D = 0.0; // corrects velocity error

    //pid correction to stay stowed
    const double STOW_P = EXTEND_DEPLOY_P;
    const double STOW_I = EXTEND_DEPLOY_I;
    const double STOW_D = EXTEND_DEPLOY_D;
    
    //trapezoidal motion profiling
    const double WRIST_MAX_VEL = 2.5; //rads per sec
    const double WRIST_MAX_ACC = 2.5; //rads per sec^2

    const double WRIST_POS_TOLERANCE = 0.1;
    const double WRIST_ABS_ENCODER_OFFSET = -1.84;

    // wrist positions in radians
    // using motor's relative encoder so assume that 0.0 is stowed
    // because it should be zeroed at stowed anyway
    const double STOWED_POS = 1.92;
    const double DEPLOYED_POS = 0.0;

    const double ROLLER_MAX_VOLTS = 1.0;
    const double WRIST_MAX_VOLTS = 5.0;
}