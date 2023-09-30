#include "util/TrapezoidalMotionProfile.h"

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC, double curPos, double setPt): 
m_maxVel{fabs(MAX_VEL)}, m_maxAcc{fabs(MAX_ACC)}{
        SetSetpoint(curPos, setPt);
}

TrapezoidalMotionProfile::TrapezoidalMotionProfile(double MAX_VEL, double MAX_ACC): 
m_maxVel{fabs(MAX_VEL)}, m_maxAcc{fabs(MAX_ACC)}{}

void TrapezoidalMotionProfile::SetSetpoint(double curPos, double setPt){
    m_setPt = setPt;
    m_targetPos = curPos;
    m_targetVel = 0.0;
    m_targetAcc = m_maxAcc;
    if (setPt < curPos)m_targetAcc *= -1;
    CalcVelTurnPos(curPos, setPt);
}

bool TrapezoidalMotionProfile::AtSetPoint(){
    if (m_targetVel == 0.0 && m_curTime != -1) return true;
    return false;
}

void TrapezoidalMotionProfile::CalcVelTurnPos(double curPos, double setPt){
    if(fabs(setPt - curPos) < m_maxVel*m_maxVel/m_maxAcc){ // for triangle motion profile
        m_velTurnPos = (m_setPt+curPos)/2;
    } else if (m_setPt > curPos)
        m_velTurnPos = m_setPt - m_maxVel*m_maxVel/(m_maxAcc*2);
    else 
        m_velTurnPos = m_setPt + m_maxVel*m_maxVel/(m_maxAcc*2);
}

void TrapezoidalMotionProfile::Periodic(){
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    
    newP += m_targetVel;
    double newTime = Utils::GetCurTimeS(), timePassed = newTime- m_curTime;
    m_curTime = newTime;

    if (m_velTurnPos < m_setPt){ // if trapezoid is pos
        if (m_targetPos > m_velTurnPos) // if after turn pt
            newV = std::max(0.0, m_targetVel - m_maxAcc * timePassed);
        else 
            newV = std::min(m_maxVel, m_targetVel + m_maxAcc * timePassed);
    } else {
        if (m_targetPos > m_velTurnPos) // if before the turn pt
            newV = std::max(-m_maxVel, m_targetVel - m_maxAcc * timePassed);
        else 
            newV = std::min(0.0, m_targetVel + m_maxAcc * timePassed);
    }

    if (newV-m_targetVel == 0) newA = 0;
    else if (newV > m_targetVel) newA = m_maxAcc;
    else newA = -m_maxAcc;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;
}

double TrapezoidalMotionProfile::GetPosition(){
    return m_targetPos;
}

double TrapezoidalMotionProfile::GetVelocity(){
    return m_targetVel;
}

double TrapezoidalMotionProfile::GetAcceleration(){
    return m_targetAcc;
}