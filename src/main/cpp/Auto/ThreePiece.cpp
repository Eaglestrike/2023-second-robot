#include "Auto/ThreePiece.h"

#include "Util/Utils.h"

using ElevatorTarget = ElevatorConstants::ElevatorTarget;
using enum ElevatorTarget;
using SwervePose = AutoPaths::SwervePose;

inline std::string toString(bool b){
    return b ? "true":"false";
}

ThreePiece::ThreePiece(ElevatorIntake &ei, AutoLineup &al, AutoPath &ap, Rollers &r):
    BaseAuto(ei, al, ap, r),
    shuff_("Three Piece", true)
{
    shuff_.add("first cone", &m_setup.firstCone, {1,1,0,0}, true);
    shuff_.add("second cone", &m_setup.secondCone, {1,1,1,0}, true);
    shuff_.add("third cone", &m_setup.thirdCone, {1,1,2,0}, true);

    shuff_.PutInteger("first height", m_targetHeights.first, {1,1,0,1});
    shuff_.PutInteger("second height", m_targetHeights.second, {1,1,1,1});
    shuff_.PutInteger("third height", m_targetHeights.third, {1,1,2,1});
    
    shuff_.PutInteger("State", m_state, {1,1,0,2});

    shuff_.PutNumber("Error Distance", 0.0, {2,1,5,2});

    shuff_.PutNumber("Target X", m_targetPose.x, {2,1,4,0});
    shuff_.PutNumber("Target Y", m_targetPose.y, {2,1,6,0});

    shuff_.PutNumber("Charge X", m_targetPoses.navChargeForward.x, {2,1,4,4});
    shuff_.PutNumber("Charge Y", m_targetPoses.navChargeForward.y, {2,1,6,4});
    
    shuff_.PutNumber("Current X", x(m_curPos), {2,1,4,1});
    shuff_.PutNumber("Current Y", y(m_curPos), {2,1,6,1});

    shuff_.PutNumber("Target Ang", m_targetPose.ang, {2,1,6,3});
}

/**
 * Setup the objects
 * 
 * 1st = preloaded object
 * 2nd = outer piece
 * 3rd = inner piece
*/
void ThreePiece::setSetup(bool firstCone, bool secondCone, bool thirdCone){
    m_setup.firstCone = firstCone;
    m_setup.secondCone = secondCone;
    m_setup.thirdCone = thirdCone;
}

/**
 * Sets the target for placing
 * Options: LOW, MID, HIGH
*/
void ThreePiece::setTarget(ElevatorTarget firstTarget, ElevatorTarget secondTarget, ElevatorTarget thirdTarget){
    m_targetHeights.first = firstTarget;
    m_targetHeights.second = secondTarget;
    m_targetHeights.third = thirdTarget;
}

void ThreePiece::Init(){
    shuff_.update(true);
    m_targetHeights.first = (ElevatorTarget)shuff_.GetInteger("second height", m_targetHeights.second);
    m_targetHeights.second = (ElevatorTarget)shuff_.GetInteger("first height", m_targetHeights.first);
    m_targetHeights.third = (ElevatorTarget)shuff_.GetInteger("third height", m_targetHeights.third);
    CalcPositions();

    m_autoStartTime = Utils::GetCurTimeS();

    m_state = FIRST_PLACE;
    m_prevState = NONE;
    m_rollersMoving = STOP;
    m_targetPose = m_targetPoses.placingFirst;

    m_ap.ResetPath();
    m_ap.Stop();
    m_ap.ResetMultiplier();
}

void ThreePiece::Periodic(){
    double curTime = Utils::GetCurTimeS();

    vec::Vector2D toTarget = vec::Vector2D{m_targetPose.x, m_targetPose.y} - m_curPos;
    double distanceToTarget = toTarget.magn();

    State tempState = m_state;
    switch(m_state){
        case NONE:
            break;
        case ALIGN_FIRST:
            if(m_prevState != m_state){
                startNewPath({m_targetPoses.placingFirst});
            }
            else if(distanceToTarget < PLACING_DISTANCE){
                m_state = FIRST_PLACE;
            }
            break;
        case FIRST_PLACE:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                ScorePos(m_setup.firstCone, m_targetHeights.first);
            }
            else if(m_ei.IsDone()){
                Outtake(m_setup.firstCone);
                if((!m_lidarData.hasCone) && (!m_lidarData.hasCube)){ //TODO could do better check
                    Stow(GOING_TO_SECOND);
                }
            }
            break;
        case GOING_TO_SECOND:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                startNewPath({m_targetPoses.navChargeForward , m_targetPoses.pickingSecond});
            }
            else if(distanceToTarget < PICKING_DISTANCE){
                m_state = PICKING_SECOND;
            }
            break;
        case PICKING_SECOND:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                Intake(m_setup.secondCone, false);
            }
            if((m_lidarData.hasCone) || (m_lidarData.hasCube)){ //TODO could do better check
                StopRollers();
                m_state = COMING_FROM_SECOND;
            }
            else if(curTime - m_stateStartTime > PICKING_TIME){
                StopRollers();
                m_state = COMING_FROM_SECOND;
                //m_state = GOING_TO_THIRD;
            }
            break;
        case COMING_FROM_SECOND:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                startNewPath({m_targetPoses.navChargeBack , m_targetPoses.placingSecond});
            }
            else if(distanceToTarget < PLACING_DISTANCE){
                m_state = SECOND_PLACE;
            }
            break;
        case SECOND_PLACE:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                ScorePos(m_setup.secondCone, m_targetHeights.second);
            }
            else if(m_ei.IsDone()){
                Outtake(m_setup.secondCone);
                if((!m_lidarData.hasCone) && (!m_lidarData.hasCube)){ //TODO could do better check
                    Stow(GOING_TO_THIRD);
                }
            }
            break;
        case GOING_TO_THIRD:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                if(m_prevState == PICKING_SECOND){
                    startNewPath({m_targetPoses.pickingThird});
                }
                else{
                    startNewPath({m_targetPoses.navChargeForward , m_targetPoses.pickingThird});
                }
            }
            else if(distanceToTarget < PICKING_DISTANCE){
                m_state = PICKING_THIRD;
            }
            break;
        case PICKING_THIRD:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                Intake(m_setup.secondCone, false);
            }
            if((m_lidarData.hasCone) || (m_lidarData.hasCube)){ //TODO could do better check
                StopRollers();
                m_state = COMING_FROM_THIRD;
            }
            else if(curTime - m_stateStartTime > PICKING_TIME){
                StopRollers();
                m_state = COMING_FROM_THIRD;
                //m_state = GOING_TO_SECOND
            }
            break;
        case COMING_FROM_THIRD:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                startNewPath({m_targetPoses.navChargeBack , m_targetPoses.placingThird});
            }
            else if(distanceToTarget < PLACING_DISTANCE){
                m_state = THIRD_PLACE;
            }
            break;
        case THIRD_PLACE:
            if(m_prevState != m_state){
                ScorePos(m_setup.thirdCone, m_targetHeights.third);
            }
            if(m_ei.IsDone()){
                Outtake(m_setup.thirdCone);
                if((!m_lidarData.hasCone) && (!m_lidarData.hasCube)){ //TODO could do better check
                    Stow(GOING_OUT);
                    std::cout<< "End time: " << curTime - m_autoStartTime <<std::endl;
                }
            }
            break;
        case GOING_OUT:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                startNewPath({m_targetPoses.navChargeForward , m_targetPoses.pickingSecond});
            }
            else if(distanceToTarget < 0.3){
                m_state = NONE;
            }
            break;
    }
    m_prevState = tempState;

    m_ei.TeleopPeriodic();
    m_ap.Periodic();
    m_r.Periodic();
    
    shuff_.PutInteger("State", m_state);

    shuff_.PutNumber("Error Distance", distanceToTarget);

    shuff_.PutNumber("Current X", x(m_curPos));
    shuff_.PutNumber("Current Y", y(m_curPos));

    shuff_.PutNumber("Target X", m_targetPose.x);
    shuff_.PutNumber("Target Y", m_targetPose.y);
    shuff_.PutNumber("Target Ang", m_targetPose.ang);

    shuff_.PutNumber("Charge X", m_targetPoses.navChargeForward.x);
    shuff_.PutNumber("Charge Y", m_targetPoses.navChargeForward.y);
}

/**
 * Called when elevator is at target: stows then moves to next state
*/
void ThreePiece::Stow(State nextState){
    if(waitStow){
        StopRollers();
        m_state = nextState;
        waitStow = false;
    }
    else{
        m_ei.Stow();
        waitStow = true;
    }
}

/**
 * Starts a new AutoPaths to go through the poses, starting where it is currently
*/
void ThreePiece::startNewPath(std::vector<SwervePose> poses){
    m_ei.Stow();
    m_ap.ResetPath();
    m_ap.AddPose(SwervePose{.time = 0.0,
                            .x = x(m_curPos), .y = y(m_curPos),
                            .vx = 0.0, .vy = 0.0,
                            .ang = m_curAng + 2.0*M_PI*m_ap.GetMultiplier(),
                            .angVel = 0.0});
    m_ap.AddPoses(poses);
    m_targetPose = poses.back();
    m_ap.StartMove();
    std::cout << "Starting new path " << m_state <<std::endl;
}

/**
 * Calls on the elevator to score
*/
void ThreePiece::ScorePos(bool cone, ElevatorTarget target){
    m_ei.SetCone(cone);
    switch(target){
        case LOW:
            m_ei.ScoreLow();
            return;
        case MID:
            m_ei.ScoreMid();
            return;
        case HIGH:
            m_ei.ScoreHigh();
            return;
        default:
            return;
    }
}

void ThreePiece::Intake(bool cone, bool flange){
    if(m_rollersMoving == INTAKE){
        return;
    }
    m_ei.SetCone(cone);
    m_r.SetCone(cone);
    m_r.Intake();
    if(flange && cone){
        m_ei.IntakeFlange();
    }
    else{
        m_ei.IntakeFromGround();
    }
    m_rollersMoving = INTAKE;
}

void ThreePiece::Outtake(bool cone){
    if(m_rollersMoving == OUTTAKE){
        return;
    }
    m_r.SetCone(cone);
    m_r.Outtake();
    m_rollersMoving = OUTTAKE;
}

void ThreePiece::StopRollers(){
    m_r.Stop();
    m_rollersMoving = STOP;
};

vec::Vector2D ThreePiece::GetDriveVel(){
    shuff_.PutString("curr vel", m_ap.GetVel().toString());
    return m_ap.GetVel();
}

double ThreePiece::GetAngVel(){
    shuff_.PutNumber("curr angVel", m_al.GetAngVel());
    return m_ap.GetAngVel();
}

bool ThreePiece::DockNow(){
    return false;
}

void ThreePiece::CalcPositions(){
    bool top = y(m_curPos) > 2.74;
    bool left = top^m_red;
    int posOffset = left? 1 : 9;

    //Placing locations
    bool coneSpots[3] {false}; //If a cone is placed on the outer column
    m_targetPoses.placingFirst = CalcScorePositions(m_setup.firstCone, m_targetHeights.first, coneSpots, posOffset);
    m_targetPoses.placingFirst.time = PLACING_TIME;
    m_targetPoses.placingSecond = CalcScorePositions(m_setup.secondCone, m_targetHeights.second, coneSpots, posOffset);
    m_targetPoses.placingThird = CalcScorePositions(m_setup.thirdCone, m_targetHeights.third, coneSpots, posOffset);

    //Picking pieces
    m_targetPoses.pickingSecond = {
        .time = TRAVEL_TIME,
        .x = PIECE_X, .y = top? PIECE_Y_4 : PIECE_Y_1,
        .vx = 0.0, .vy = 0.0,
        .ang = left? -0.2 + 2*M_PI: 0.2, .angVel = 0.0
    };
    m_targetPoses.pickingThird = {
        .time = TRAVEL_TIME,
        .x = PIECE_X, .y = top? PIECE_Y_3 : PIECE_Y_2,
        .vx = 0.0, .vy = 0.0,
        .ang = left? -1.04 + 2*M_PI: 1.04, .angVel = 0.0
    };
    if(m_red){
        m_targetPoses.pickingSecond = Utils::GetRedPose(m_targetPoses.pickingSecond);
        m_targetPoses.pickingThird = Utils::GetRedPose(m_targetPoses.pickingThird);
    }

    //Navigating Charge station
    m_targetPoses.navChargeForward = {
        .time = CHARGETIME_FORWARD,
        .x = CHARGE_X, .y = CHARGE_Y + (top? NAV_WIDTH : -NAV_WIDTH),
        .vx = NAV_VEL, .vy = 0.0,
        .ang = left? -M_PI/2.0 + 2*M_PI: M_PI/2.0, .angVel = 0.0
    };
    m_targetPoses.navChargeBack = {
        .time = CHARGETIME_BACK,
        .x = CHARGE_X, .y = CHARGE_Y + (top? NAV_WIDTH : -NAV_WIDTH),
        .vx = -NAV_VEL, .vy = 0.0,
        .ang = left? -M_PI/2.0 + 2*M_PI: M_PI/2.0, .angVel = 0.0
    };
    if(m_red){
        m_targetPoses.navChargeForward = Utils::GetRedPose(m_targetPoses.navChargeForward);
        m_targetPoses.navChargeBack = Utils::GetRedPose(m_targetPoses.navChargeBack);
    }

    std::cout<<"left "<<toString(left)<<std::endl;
}

/**
 * Calculates the poses to score on a grid
 * Has some logic to resolve 2 cone high/mid/low
*/
SwervePose ThreePiece::CalcScorePositions(bool cone, ElevatorTarget target, bool coneSpots[3], int posOffset){
    double forwardAng = m_red ? M_PI : 0.0;
    int toCenter = posOffset < 4 ? 1 : -1;
    int height = std::max(Utils::getPieceHeight(target), 1);
    int targPos;
    if(m_setup.firstCone){
        targPos = (coneSpots[height-1]? 2 : 0)*toCenter + posOffset;
        coneSpots[height-1] = true;
    }
    else{
        targPos = posOffset + toCenter;
    }
    FieldConstants::ScorePair score = Utils::GetScoringPos(targPos, height, m_red);
    return {
        .time = TRAVEL_TIME,
        .x = x(score.first), .y = y(score.first),
        .vx = 0.0, .vy = 0.0,
        .ang = forwardAng + M_PI, .angVel = 0.0
    };
}