#pragma once 

namespace IntakeConstants{
    const int WRIST_MOTOR_ID = 4;
    const int ROLLER_MOTOR_ID = 31;
    const int WRIST_ENCODER_CAN_ID = 0;

    //EXTEND DEPLOY FEEDFORWARD-PID

    //feedforward constants
    const double EXTEND_DEPLOY_S = 0.05; // volts
    const double EXTEND_DEPLOY_G = 0.275; // volts needed to resist gravity
    const double EXTEND_DEPLOY_V = 1.0; // volts*seconds/rad
    const double EXTEND_DEPLOY_A = 0.04; // volts*seconds^2/rad

    //pid accounts for velocity and position error
    const double EXTEND_DEPLOY_P = 1.75; // corrects position error
    const double EXTEND_DEPLOY_I = 0.1; // accounts for cumulative pos err
    const double EXTEND_DEPLOY_D = 0.1; // corrects velocity error

    //pid correction to stay stowed
    const double STOW_P = EXTEND_DEPLOY_P;
    const double STOW_I = EXTEND_DEPLOY_I;
    const double STOW_D = EXTEND_DEPLOY_D;
    
    //trapezoidal motion profiling
    const double WRIST_MAX_VEL = 5; //rads per sec
    const double WRIST_MAX_ACC = 10; //rads per sec^2

    const double WRIST_POS_TOLERANCE = 0.05;
    const double WRIST_ABS_ENCODER_OFFSET = -1.84-0.094;

    // wrist positions in radians
    // using motor's relative encoder so assume that 0.0 is stowed
    // because it should be zeroed at stowed anyway
    const double STOWED_POS = 1.92;
    const double DEPLOYED_POS = 0.0;

    const double ROLLER_MAX_VOLTS = 3.0;
    const double WRIST_MAX_VOLTS = 5.0;
}