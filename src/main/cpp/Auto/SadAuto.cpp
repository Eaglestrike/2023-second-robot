#include "Auto/SadAuto.h"

SadAuto::SadAuto(SwerveControl& swerve_control, ElevatorIntake& elevator_intake): m_swerveDrivebase(swerve_control), m_ei(elevator_intake) {
    m_ei.Init();
    m_state = NOT_STARTED;
}

void SadAuto::Start() {
    m_state = PLACING_UP;
}

void SadAuto::Periodic() {
    if (m_state == NOT_STARTED) {
        return;
    }

    frc::SmartDashboard::PutString("sad auto state", StateToString(m_state));

    m_ei.TeleopPeriodic();

    switch (m_state) {
        case NOT_STARTED:
            return;
        case PLACING_UP:
            m_ei.ScoreLow();

            if (m_ei.IsDone()) {
                m_state = PLACING_DOWN;
            }
            break;
        case PLACING_DOWN:
            m_state = PLACING_ROLLERS;
            break;
        case PLACING_ROLLERS:
            m_ei.ToggleRoller(true);
            if (m_ei.IsDone()) {
                m_state = STOWING;
            }
            break;
        case STOWING:
            m_ei.Stow();

            if (m_ei.IsDone()) {
                m_state = MOVING;
                timer.Start();
            }

            break;
        
        case MOVING:
            m_swerveDrivebase.SetRobotVelocity(target, 0, 0, timer.Get().value() - prevTime);

            if (timer.Get().value() > time_with_velocity) {
                m_state = DONE;
                m_swerveDrivebase.SetRobotVelocity({0.0, 0.0}, 0, 0, 0);
            }
            prevTime = timer.Get().value();

            break;
        
        case DONE:
            m_ei.Kill();
    }
}

SadAuto::State SadAuto::GetCurState() {
    return m_state;
}

vec::Vector2D SadAuto::GetVelocity() {
    return m_swerveDrivebase.GetRobotVelocity(0.0);
}

std::string SadAuto::StateToString(State state) {
    switch (state) {
        case NOT_STARTED:
            return "NOT STARTED";
        case PLACING_UP:
            return "PLACING UP";
        case PLACING_DOWN:
            return "PLACING DOWN";
        case PLACING_ROLLERS:
            return "PLACING ROLLERS";
        case STOWING:
            return "STOWING";
        case MOVING:
            return "MOVING";
        case DONE:
            return "DONE";
        default:
            return "state unknown";
    }
}

void SadAuto::debugChangeTime(double new_time) {
    if (new_time != time_with_velocity) {
        time_with_velocity = new_time;
    }
}