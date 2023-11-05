#include "Auto/ThreePiece.h"

#include "Util/MathUtil.h"

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

void ThreePiece::Init(){
    m_autoStartTime = Utils::GetCurTimeS();

    m_state = GOING_FIRST;
    m_prevState = NONE;
    m_stateStartTime = m_autoStartTime;
}

void ThreePiece::Periodic(){
    bool lineupAtTarget = m_al.GetPosExecuteState() == m_al.AT_TARGET;
    bool autoPathAtTarget = m_ap.GetExecuteState() == m_ap.AT_TARGET;
    
    State tempState = m_state;
    switch(m_state){
        case NONE:
            break;
        case GOING_FIRST:
            if(m_prevState != m_state){
                m_al.SetTarget();
                m_al.GetVel();
            }
            if(lineupAtTarget){
                m_state = FIRST_PLACE;
            }
            break;
        case FIRST_PLACE:

        case GOING_TO_SECOND:

        case PICKING_SECOND:

        case COMING_FROM_SECOND:

        case SECOND_PLACE:

        case GOING_TO_THIRD:

        case PICKING_THIRD:

        case COMING_FROM_THIRD:

        case THIRD_PLACE:

        case GOING_OUT:
    }
    m_prevState = tempState;
}


vec::Vector2D ThreePiece::GetDriveVel(){

}

double ThreePiece::GetAngVel(){

}

bool ThreePiece::DockNow(){

}

