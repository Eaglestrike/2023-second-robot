#pragma once 

namespace IntakeConstants{
    const int WRIST_MOTOR_ID = 0;
    const int ROLLER_MOTOR_ID = 0;
    const int WRIST_ENCODER_PIN = 0;

    //EXTEND DEPLOY FEEDFORWARD-PID

    //feedforward constants
    const double EXTEND_DEPLOY_S = 0.0; // volts
    const double EXTEND_DEPLOY_G = 0.0; // volts needed to resist gravity
    const double EXTEND_DEPLOY_V = 0.0; // volts*seconds/rad
    const double EXTEND_DEPLOY_A = 0.0; // volts*seconds^2/rad

    //pid accounts for velocity and position error
    const double EXTEND_DEPLOY_P = 0.0; // corrects position error
    const double EXTEND_DEPLOY_I = 0.0; // accounts for cumulative pos err
    const double EXTEND_DEPLOY_D = 0.0; // corrects velocity error
    
    //trapezoidal motion profiling
    const double WRIST_MAX_VEL = 0.0; //rads per sec
    const double WRIST_MAX_ACC = 0.0; //rads per sec^2

    const double WRIST_POS_TOLERANCE = 0.0;
    const double WRIST_ABS_ENCODER_OFFSET = 0.0;

    // wrist positions in radians
    // using motor's relative encoder so assume that 0.0 is stowed
    // because it should be zeroed at stowed anyway
    const double STOWED_POS = 0.0;
    const double DEPLOYED_POS = 0.0;

    const double ROLLER_MAX_VOLTS = 0.0;
    const double WRIST_MAX_VOLTS = 0.0;
}