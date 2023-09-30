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
    if (setPt < curPos) m_targetAcc *= -1;
    CalcTurnTime(curPos, setPt);
}

bool TrapezoidalMotionProfile::AtSetPoint(){
    if (Utils::GetCurTimeS() > m_endTime) return true;
    return false;
}

void TrapezoidalMotionProfile::CalcTurnTime(double curPos, double setPt){
    m_curTime = Utils::GetCurTimeS();
    if(fabs(setPt - curPos) < m_maxVel*m_maxVel/m_maxAcc){ // for triangle motion profile
        double halfTriangleBase = std::sqrt(fabs(setPt - curPos)/m_maxAcc);
        m_turnTime = halfTriangleBase + m_curTime;
        m_endTime = halfTriangleBase*2 + m_curTime;
    } else {// for trapezoid
        m_turnTime = fabs(m_setPt - curPos)/m_maxVel + m_curTime;
        m_endTime = m_turnTime + m_maxVel/m_maxAcc;
    }
}

void TrapezoidalMotionProfile::Calculate(){
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;

    newP += m_targetVel;

    double curTime = Utils::GetCurTimeS();
    m_curTime = curTime;
    double timePassed = curTime - m_curTime;

    if (m_targetPos < m_setPt){ // if trapezoid is pos
        if (curTime > m_turnTime) // if after turn time
            newV = std::max(0.0, m_targetVel - m_maxAcc * timePassed);
        else 
            newV = std::min(m_maxVel, m_targetVel + m_maxAcc * timePassed);
    } else {
        if (curTime < m_turnTime)  // if before the turn time
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