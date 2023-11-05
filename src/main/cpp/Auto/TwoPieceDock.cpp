#include "Auto/TwoPieceDock.h"

#include "Util/Mathutil.h"
#include "GeneralConstants.h"

void TwoPieceDock::Init() {
    m_ei.Init();
    m_state = PLACE1_UP;
    m_ei.SetCone(true);
    m_r.SetCone(true);
    m_ei.ScoreHigh();
}

void TwoPieceDock::Periodic() {
    double curTime = Utils::GetCurTimeS();

    switch (m_state) {
        case NOT_STARTED:
            break;
        case PLACE1_UP:
            if (m_ei.IsDone()) {
                m_state = PLACE1_HOLD;
                m_r.Outtake();
                m_startTime = Utils::GetCurTimeS();
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
                m_ap.AddPose({curTime, m_curPos.x(), m_curPos.y(), 0, 0, m_curAng, 0});
                m_ap.AddPose({curTime + 3, 7.085, 0.923, 0, 0, 0.124, 0});
                m_ap.StartMove();
            }
            break;
        case GO_TO_PIECE_2:
            if (curTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET) {
                m_ei.SetCone(false);
                m_r.SetCone(false);
                m_ei.IntakeFromGround();
                m_r.Intake();
                m_state = INTAKE2;
                m_startTime = Utils::GetCurTimeS();
            }
            break;
        case INTAKE2:
            if (curTime - m_startTime > INTAKE_CUBE_TIME) {
                m_ei.Stow();
                m_state = GO_TO_GRID_2;
                m_ap.ResetPath();
                m_ap.AddPose({curTime, m_curPos.x(), m_curPos.y(), 0, 0, m_curAng, 0});
                m_ap.AddPose({curTime + 3, 1.93, 1.07, 0, 0, M_PI, 0});
                m_ap.StartMove();
            }
            break;
        case GO_TO_GRID_2:
            if (curTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET) {
                m_ei.ScoreHigh();
                m_state = PLACE2_UP;
            }
            break;
        case PLACE2_UP:
            if (m_ei.IsDone()) {
                m_state = PLACE2_HOLD;
                m_r.Outtake();
                m_startTime = Utils::GetCurTimeS();
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
                m_ap.AddPose({curTime, m_curPos.x(), m_curPos.y(), 0, 0, m_curAng, 0});
                m_ap.AddPose({curTime + 1.5, 4.525, 0.6, 1, 0, 0, 0});
                m_ap.AddPose({curTime + 3, 7.16, 2.18, 0, 0, 0.55, 0});
                m_ap.StartMove();
            }
            break;
        case GO_TO_PIECE_3:
            if (curTime > m_ap.GreatestTime() + SPLINE_TIME_OFFSET) {
                m_ei.IntakeFromGround();
                m_r.Intake();
                m_state = INTAKE3;
                m_startTime = Utils::GetCurTimeS();
            }
            break;
        case INTAKE3:
            if (curTime - m_startTime > INTAKE_CUBE_TIME) {
                m_ei.Stow();
                m_state = GO_TO_DOCK;
                m_ap.ResetPath();
                m_ap.AddPose({curTime, m_curPos.x(), m_curPos.y(), 0, 0, m_curAng, 0});
                m_ap.AddPose({curTime + 0.7, 6, 2.13, -AutoConstants::PRE_DOCK_SPEED, 0, M_PI / 2, 0});
                m_ap.StartMove();
            }
            break;
        case GO_TO_DOCK:
            if (curTime > m_ap.GreatestTime()) {
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