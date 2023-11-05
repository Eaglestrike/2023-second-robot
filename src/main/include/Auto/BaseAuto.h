#pragma once

#include "Drive/AutoLineup.h"
#include "Drive/AutoPath.h"
#include "Elevator/ElevatorIntake.h"
#include "Elevator/Intake/Rollers.h"
#include "Elevator/Lidar/LidarReader.h"

#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

class BaseAuto {
public:
    BaseAuto(ElevatorIntake &ei, AutoLineup &al, AutoPath &ap, Rollers &r);

    virtual void Init();
    virtual void Periodic();
    void UpdateOdom(vec::Vector2D pos, double ang, vec::Vector2D wheelVel, double tilt, LidarReader::LidarData lidarData);

    virtual vec::Vector2D GetDriveVel();
    virtual double GetAngVel();

protected:
    vec::Vector2D m_curPos; 
    double m_curAng; 
    vec::Vector2D m_curWheelVel;
    double m_curTilt;
    LidarReader::LidarData m_lidarData;

    ElevatorIntake &m_ei;
    AutoLineup &m_al;
    AutoPath &m_ap;
    Rollers &m_r;
};