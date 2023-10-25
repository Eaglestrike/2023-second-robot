namespace IntakeElevatorConstants{
    struct ElevatorIntakePosInfo{
        double ELEVATOR_LENG; //vir units
        double INTAKE_ANGLE; //rads
    };

    struct GamePieceInfo{
        ElevatorIntakePosInfo SCORE_LOW;
        ElevatorIntakePosInfo SCORE_MID;
        ElevatorIntakePosInfo SCORE_HIGH;
        ElevatorIntakePosInfo GROUND_INTAKE;
        ElevatorIntakePosInfo HP_INTAKE;
    };

    const GamePieceInfo coneScoreInfo{{0.0, 0.58},
                                      {0.316, 0.73},
                                      {0.545, 0.8},
                                      {0.0, 0.0}, // ground intake
                                      {0.0, 1.5}}; // intake from drop station

    const GamePieceInfo cubeScoreInfo{{0.0, 0.58},
                                      {0.291, 1.25},
                                      {0.573, 0.8}, // elevator pos seems wayyy to high
                                      {0.0, 0.13}, // intake from ground
                                      {0.0, 0.0}}; 
}

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
    
    //trapezoidal motion profiling
    const double WRIST_MAX_VEL = 5; //rads per sec
    const double WRIST_MAX_ACC = 10; //rads per sec^2

    // all in rads
    const double WRIST_POS_TOLERANCE = 0.05;
    const double WRIST_ABS_ENCODER_OFFSET = - 1.84 - 0.094; //cad says 0.094 should b 0.034 but idk

    // wrist positions in radians, 0.0 is parallel to ground and flipping intake up is positive
    const double MAX_POS = 1.85;
    const double STOWED_POS = 1.84;
    const double INTAKE_UPRIGHT_ANGLE = 1.4;


    //todo:
    const double MIN_POS = 0.0;
    const double DEPLOYED_POS = 0.0;

    // const double NORMAL_CURRENT = 15; //acc idk
     struct GamePieceInfo {
        double SPIKE_CURRENT;
        double KEEP_VOLTS;
        double OUT_VOLTS;
    };

    const GamePieceInfo CONE_INFO = {100.0,
                                     -1.5,
                                     3.0};
    const GamePieceInfo CUBE_INFO = {15.0,
                                     0.7,
                                     3.0}; // MAYBE LESS
    
    // const double KEEP_CONE_CURRENT = 5.0;
    // const double KEEP_CUBE_CURRENT = 10.0;

    const double ROLLER_MAX_VOLTS = 4.0;
    const double WRIST_MAX_VOLTS = 5.0;
}