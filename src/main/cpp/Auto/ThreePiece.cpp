#include "Auto/ThreePiece.h"

#include "Util/Utils.h"

using ElevatorTarget = ElevatorConstants::ElevatorTarget;
using enum ElevatorTarget;
using SwervePose = AutoPaths::SwervePose;

inline std::string toString(bool b){
    return b ? "true":"false";
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

void ThreePiece::setTarget(ElevatorTarget firstTarget, ElevatorTarget secondTarget, ElevatorTarget thirdTarget){
    m_targetHeights.first = firstTarget;
    m_targetHeights.second = secondTarget;
    m_targetHeights.third = thirdTarget;
}

void ThreePiece::Init(){
    CalcPositions();

    m_autoStartTime = Utils::GetCurTimeS();

    m_state = ALIGN_FIRST;
    m_prevState = NONE;
    m_targetPose = m_targetPoses.placingFirst;
    m_stateStartTime = m_autoStartTime;
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
            else if(distanceToTarget < 0.3){
                m_state = FIRST_PLACE;
            }
            break;
        case FIRST_PLACE:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                ScorePos(m_setup.firstCone, m_targetHeights.first);
            }
            else if(m_ei.IsDone()){
                m_r.Outtake();
            }
            if((!m_lidarData.hasCone) || (!m_lidarData.hasCube)){ //TODO could do better check
                m_state = GOING_TO_SECOND;
            }
            break;
        case GOING_TO_SECOND:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                startNewPath({m_targetPoses.navChargeForward , m_targetPoses.pickingSecond});
            }
            else if(distanceToTarget < 0.3){
                m_state = PICKING_SECOND;
            }
            break;
        case PICKING_SECOND:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                m_ei.SetCone(m_setup.secondCone);
                m_r.SetCone(m_setup.secondCone);
                m_ei.IntakeFromGround();
            }
            else if(m_ei.IsDone()){
                m_r.Intake();
            }
            if((!m_lidarData.hasCone) || (!m_lidarData.hasCube)){ //TODO could do better check
                m_state = COMING_FROM_SECOND;
            }
            else if(curTime - m_stateStartTime > 3.0){
                m_state = GOING_TO_THIRD;
            }
            break;
        case COMING_FROM_SECOND:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                startNewPath({m_targetPoses.navChargeBack , m_targetPoses.placingSecond});
            }
            else if(distanceToTarget < 0.3){
                m_state = SECOND_PLACE;
            }
            break;
        case SECOND_PLACE:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                ScorePos(m_setup.secondCone, m_targetHeights.second);
            }
            else if(m_ei.IsDone()){
                m_r.Outtake();
            }
            if((!m_lidarData.hasCone) || (!m_lidarData.hasCube)){ //TODO could do better check
                m_state = GOING_TO_THIRD;
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
            else if(distanceToTarget < 0.3){
                m_state = PICKING_THIRD;
            }
            break;
        case PICKING_THIRD:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                m_ei.SetCone(m_setup.secondCone);
                m_r.SetCone(m_setup.secondCone);
                m_ei.IntakeFromGround();
            }
            else if(m_ei.IsDone()){
                m_r.Intake();
            }
            if((!m_lidarData.hasCone) || (!m_lidarData.hasCube)){ //TODO could do better check
                m_state = COMING_FROM_THIRD;
            }
            break;
        case COMING_FROM_THIRD:
            if(m_prevState != m_state){
                m_stateStartTime = curTime;
                startNewPath({m_targetPoses.navChargeBack , m_targetPoses.placingThird});
            }
            else if(distanceToTarget < 0.3){
                m_state = THIRD_PLACE;
            }
            break;
        case THIRD_PLACE:
            if(m_prevState != m_state){
                ScorePos(m_setup.thirdCone, m_targetHeights.third);
            }
            if(m_ei.IsDone()){
                m_r.Outtake();
            }
            if((!m_lidarData.hasCone) || (!m_lidarData.hasCube)){ //TODO could do better check
                m_state = GOING_OUT;
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
}

void ThreePiece::startNewPath(std::vector<SwervePose> poses){
    m_ei.Stow();
    m_ap.ResetPath();
    m_ap.AddPoses(poses);
    m_ap.StartMove();
    m_targetPose = poses.back();
}

void ThreePiece::ScorePos(bool cone, ElevatorTarget target){
    m_ei.SetCone(cone);
    m_r.SetCone(cone);
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


vec::Vector2D ThreePiece::GetDriveVel(){
    return m_ap.GetVel();
}

double ThreePiece::GetAngVel(){
    return m_al.GetAngVel();
}

bool ThreePiece::DockNow(){
    return false;
}

void ThreePiece::CalcPositions(){
    double forwardAng = m_red ? M_PI : 0.0;
    bool top = y(m_curPos) > 2.74;
    bool left = top^m_red;
    int posOffset = left? 1 : 9;
    printf("top: %s, left: %s, offset: %d", toString(top).data(), toString(left).data(), posOffset);

    //Placing locations
    bool coneSpots[3] {false}; //If a cone is placed on the outer column
    m_targetPoses.placingFirst = CalcScorePositions(m_setup.firstCone, m_targetHeights.first, coneSpots, posOffset);
    m_targetPoses.placingSecond = CalcScorePositions(m_setup.secondCone, m_targetHeights.second, coneSpots, posOffset);
    m_targetPoses.placingThird = CalcScorePositions(m_setup.thirdCone, m_targetHeights.third, coneSpots, posOffset);

    //Picking pieces
    m_targetPoses.pickingSecond = {
        .time = 0.0,
        .x = PIECE_X, .y = top? PIECE_Y_4 : PIECE_Y_1,
        .vx = 0.0, .vy = 0.0,
        .ang = forwardAng, .angVel = 0.0
    };
    m_targetPoses.pickingThird = {
        .time = 0.0,
        .x = PIECE_X, .y = top? PIECE_Y_3 : PIECE_Y_2,
        .vx = 0.0, .vy = 0.0,
        .ang = forwardAng + left? -60.0 : 60.0, .angVel = 0.0
    };
    if(m_red){
        m_targetPoses.pickingSecond = Utils::GetRedPose(m_targetPoses.pickingSecond);
        m_targetPoses.pickingThird = Utils::GetRedPose(m_targetPoses.pickingThird);
    }

    //Navigating Charge station
    m_targetPoses.navChargeForward = {
        .time = 0.0,
        .x = CHARGE_X, .y = CHARGE_Y + (top? NAV_WIDTH : -NAV_WIDTH),
        .vx = NAV_VEL, .vy = 0.0,
        .ang = forwardAng, .angVel = 0.0
    };
    if(m_red){
        m_targetPoses.navChargeForward = Utils::GetRedPose(m_targetPoses.navChargeForward);
    }
    m_targetPoses.navChargeBack = m_targetPoses.navChargeForward;
    m_targetPoses.navChargeBack.vx *= -1.0;
    m_targetPoses.navChargeBack.vy *= -1.0;
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
    printf("target: %d, %d, position: %s", targPos, height, score.first.toString().data());
    return {
        .time = 0.0,
        .x = x(score.first), .y = y(score.first),
        .vx = 0.0, .vy = 0.0,
        .ang = forwardAng + M_PI, .angVel = 0.0
    };
}