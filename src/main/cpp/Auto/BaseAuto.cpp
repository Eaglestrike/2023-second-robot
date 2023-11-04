#include "Auto/BaseAuto.h"

BaseAuto::BaseAuto(ElevatorIntake &ei, AutoLineup &al, AutoPath &ap)
    : m_curAng{0}, m_curTilt{0}, m_ei{ei}, m_al{al}, m_ap{ap} {}

void BaseAuto::UpdateOdom(vec::Vector2D pos, double ang, vec::Vector2D wheelVel, double tilt) {
    m_curPos = pos;
    m_curAng = ang;
    m_curWheelVel = wheelVel;
    m_curTilt = tilt;
}