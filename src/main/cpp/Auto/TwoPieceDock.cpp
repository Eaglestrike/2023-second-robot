#include "Auto/TwoPieceDock.h"

#include "Util/Utils.h"
#include "GeneralConstants.h"

#include <frc/smartdashboard/SmartDashboard.h>

void TwoPieceDock::Init() {
    m_ei.Init();
    m_state = PLACE1_UP;
    m_ei.SetCone(true);
    m_r.SetCone(true);
    m_ei.ScoreHigh();

    m_doOnce = false;

    // sorry for baad code
    PIECE_2_GND = ORIG_PIECE_2_GND;
    PIECE_2_SCORE = ORIG_PIECE_2_SCORE;
    PIECE_3_MID = ORIG_PIECE_3_MID;
    PIECE_3_GND = ORIG_PIECE_3_GND;
    PRE_DOCK = ORIG_PRE_DOCK;

    if (m_red) {
        PIECE_2_GND = Utils::GetRedPose(PIECE_2_GND);
        PIECE_2_SCORE = Utils::GetRedPose(PIECE_2_SCORE);
        PIECE_3_MID = Utils::GetRedPose(PIECE_3_MID);
        PIECE_3_GND = Utils::GetRedPose(PIECE_3_GND);
        PRE_DOCK = Utils::GetRedPose(PRE_DOCK);
    }
}

void TwoPieceDock::Periodic() {
    double curTime = Utils::GetCurTimeS();

    // m_ei.UpdateLidarData(m_lidarData);
    // m_ap.UpdateOdom(m_curPos, m_curAng, m_curWheelVel);
    // m_r.UpdateLidarData(m_lidarData);

    frc::SmartDashboard::PutNumber("2piece state", m_state);

    switch (m_state) {
        case NOT_STARTED:
            break;
        case PLACE1_UP:
            if (m_ei.IsDone()) {
                m_state = PLACE1_HOLD;
                m_r.Outtake();
                m_startTime = curTime;
            }
            break;
        case PLACE1_HOLD:
            if (curTime - m_startTime > ROLLER_OUTTAKE_TIME) {
                m_ei.Stow();
                m_r.Stop();
                m_state = PLACE1_DOWN;
            }
            break;
        case PLACE1_DOWN:
            if (m_ei.IsDone()) {
                m_state = GO_TO_PIECE_2;
                m_ap.ResetPath();
                m_ap.AddPose({0, m_curPos.x(), m_curPos.y(), 0, 0, GetAbsAng(), 0});
                m_ap.AddPose(PIECE_2_GND);
                m_ap.StartMove();
                m_startTime = curTime;
            }
            break;
        case GO_TO_PIECE_2:
            if (curTime - m_startTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET - 1 && !m_doOnce) {
                m_doOnce = true;
                m_ei.SetCone(false);
                m_r.SetCone(false);
                m_ei.IntakeFromGround();
                m_r.Intake();
            }
            if (curTime - m_startTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET) {
                m_doOnce = false;
                m_state = INTAKE2;
                m_startTime = curTime;
            }
            break;
        case INTAKE2:
            if (curTime - m_startTime > INTAKE_CUBE_TIME) {
                m_ei.Stow();
                m_state = GO_TO_GRID_2;
                m_ap.ResetPath();
                m_ap.AddPose({0, m_curPos.x(), m_curPos.y(), 0, 0, GetAbsAng(), 0});
                // IF WRONG, FLIP
                if (m_lidarData.hasCube) {
                    if (m_red) {
                        PIECE_2_SCORE.y -= m_lidarData.cubePos / 100.0;
                    } else {
                        PIECE_2_SCORE.y += m_lidarData.cubePos / 100.0;
                    }
                }
                m_startTime = curTime;
                m_ap.AddPose(PIECE_2_SCORE);
                m_ap.StartMove();
            }
            break;
        case GO_TO_GRID_2:
            if (curTime - m_startTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET - 0.5 && !m_doOnce) {
                m_doOnce = true;
                m_ei.ScoreHigh();
            }
            if (curTime - m_startTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET) {
                m_doOnce = false;
                m_state = PLACE2_UP;
            }
            break;
        case PLACE2_UP:
            if (m_ei.IsDone()) {
                m_state = PLACE2_HOLD;
                m_r.Outtake();
                m_startTime = curTime;
            }
            break;
        case PLACE2_HOLD:
            if (curTime - m_startTime > ROLLER_OUTTAKE_TIME) {
                m_ei.Stow();
                m_r.Stop();
                m_state = PLACE2_DOWN;
            }
            break;
        case PLACE2_DOWN:
            if (m_ei.IsDone()) {
                m_state = GO_TO_PIECE_3;
                m_ap.ResetPath();
                m_ap.AddPose({0, m_curPos.x(), m_curPos.y(), 0, 0, GetAbsAng(), 0});
                m_ap.AddPose(PIECE_3_MID);
                m_ap.AddPose(PIECE_3_GND);
                m_startTime = curTime;
                m_ap.StartMove();
            }
            break;
        case GO_TO_PIECE_3:
            if (curTime - m_startTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET - 1 && !m_doOnce) {
                m_ei.IntakeFromGround();
                m_r.Intake();
                m_doOnce = true;
            }
            if (curTime - m_startTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET) {
                m_doOnce = false;
                m_state = INTAKE3;
                m_startTime = curTime;
            }
            break;
        case INTAKE3:
            if (curTime - m_startTime > INTAKE_CUBE_TIME) {
                m_ei.Stow();
                m_state = GO_TO_DOCK;
                m_ap.ResetPath();
                m_ap.AddPose({0, m_curPos.x(), m_curPos.y(), 0, 0, GetAbsAng(), 0});
                m_ap.AddPose(PRE_DOCK);
                m_ap.StartMove();
                m_startTime = curTime;
            }
            break;
        case GO_TO_DOCK:
            if (curTime - m_startTime > m_ap.GreatestTime()) {
                m_state = CAN_DOCK;
            }
            break;
        case CAN_DOCK:
            break;
    }

    m_ei.Periodic();
    m_ei.TeleopPeriodic();
    m_ap.Periodic();
    m_r.Periodic();
}

vec::Vector2D TwoPieceDock::GetDriveVel() {
    if (m_state == GO_TO_PIECE_2 || m_state == GO_TO_GRID_2 || m_state == GO_TO_PIECE_3 || m_state == GO_TO_DOCK) {
        return m_ap.GetVel();
    }
    return {0, 0};
}

double TwoPieceDock::GetAngVel() {
    if (m_state == GO_TO_PIECE_2 || m_state == GO_TO_GRID_2 || m_state == GO_TO_PIECE_3 || m_state == GO_TO_DOCK) {
        return m_ap.GetAngVel();
    }
    return 0;
}

bool TwoPieceDock::DockNow() {
    return m_state == CAN_DOCK;
}

double TwoPieceDock::GetAbsAng() {
    return m_curAng + m_ap.GetMultiplier() * M_PI * 2;
}