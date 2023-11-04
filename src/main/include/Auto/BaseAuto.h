#pragma once

#include "Drive/AutoLineup.h"
#include "Drive/AutoPath.h"
#include "Elevator/ElevatorIntake.h"

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

class BaseAuto {
public:
    BaseAuto();

    virtual void Init();
    virtual void Periodic();
    virtual void UpdateOdom(vec::Vector2D pos, double ang, vec::Vector2D wheelVel, double tilt);

    virtual vec::Vector2D GetDriveVel();
    virtual double GetAngVel();

private:
    vec::Vector2D m_curPos; 
    double m_curAng; 

    ElevatorIntake &m_ei;
    AutoLineup &m_al;
    AutoPath &m_ap;
};