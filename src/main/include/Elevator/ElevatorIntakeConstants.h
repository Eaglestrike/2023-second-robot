namespace IntakeElevatorConstants{
     //all angles in radians, all lengths in meters

    struct ScoreInfo{
        double ELEVATOR_LENG;
        double INTAKE_ANGLE; // how far out the point of contact between gamepiece and intake is from the intake's point of rotation
    };

    struct GamePieceInfo{
        ScoreInfo LOW_INFO;
        ScoreInfo MID_INFO;
        ScoreInfo HIGH_INFO;
    };

    const GamePieceInfo coneScoreInfo{{0.0, 0.0},
                                      {0.0, 0.0},
                                      {0.0, 0.0}}; 

     const GamePieceInfo cubeScoreInfo{{0.0, 0.0},
                                      {0.0, 0.0},
                                      {0.0, 0.0}}; 
    
    // const double INTAKE_BAR_LENGTH = 0.0;
    // const double ELEVATOR_ANGLE = 0.0; // the acute angle elevator makes w the drivetrain
    
    // const double CUBE_INTAKE_SCORE_ANGLE = 0.0;

    //scoring heights in meters from ground
    // const double HIGH_CONE_HEIGHT = 0.0;
    // const double HIGH_CUBE_HEIGHT = 0.0;
    // const double MID_CONE_HEIGHT = 0.0;
    // const double MID_CUBE_HEIGHT = 0.0;
    // const double LOW_HEIGHT = 0.0;
}

namespace ElevatorConstants{
    const double MAX_POS = 0.0;
    const double MIN_POS = 0.0;
}

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
    const double WRIST_ABS_ENCODER_OFFSET = -1.84 -0.094; //cad says its 0.034 tho

    // wrist positions in radians
    // using motor's relative encoder so assume that 0.0 is stowed
    // because it should be zeroed at stowed anyway
    const double MAX_POS = 1.84;
    //todo:
    const double MIN_POS = 0.0;
    const double STOWED_POS = 1.92;
    const double DEPLOYED_POS = 0.0;
    const double INTAKE_UPRIGHT_ANGLE = 0.0;
    


    const double ROLLER_MAX_VOLTS = 1.0;
    const double WRIST_MAX_VOLTS = 5.0;
}