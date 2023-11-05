#include "Auto/SadAuto.h"

SadAuto::SadAuto(ElevatorIntake& elevator_intake, Rollers& roller): m_ei(elevator_intake), m_rollers(roller) {
    // m_ei.Init();
    // m_swerveDrivebase.Init();
    m_state = NOT_STARTED;
    std::cout << "sad auto constructor finished running" << "\n";
    frc::SmartDashboard::PutString("sad auto state", StateToString(m_state));
    frc::SmartDashboard::PutNumber("sad auto timer value", timer.Get().value());
}

void SadAuto::Start() {
    m_state = PLACING_UP;
    m_ei.SetCone(true);
    m_ei.ScoreLow();
    std::cout << "sad auto start finished running" << "\n";
}

void SadAuto::Periodic() {
    frc::SmartDashboard::PutString("sad auto state", StateToString(m_state));
    frc::SmartDashboard::PutNumber("sad auto timer value", timer.Get().value());

    std::cout << "sad auto periodic running" << "\n";

    m_ei.TeleopPeriodic();
    m_rollers.Periodic();

    switch (m_state) {
        case NOT_STARTED:
            Start();
            return;
        case PLACING_UP:
            std::cout << "sad auto placing up" << "\n";
            if (m_ei.IsDone()) {
                m_state = PLACING_ROLLERS;
                m_rollers.Outtake();
                timer.Start();
            }
            break;
        case PLACING_ROLLERS:
            std::cout << "sad auto placing rollers" << "\n";
            if (timer.Get().value() > time_outtaking) {
                m_state = STOWING;
                m_rollers.Stop();
                m_ei.Stow();
                timer.Stop();
                timer.Reset();
            }
            break;
        case STOWING:
            std::cout << "sad auto stowing" << "\n";
            if (m_ei.IsDone()) {
                m_state = MOVING;
                timer.Start();
            }
            break;
        
        case MOVING:
            std::cout << "sad auto moving" << "\n";
            m_vel = target;

            if (timer.Get().value() > time_with_velocity) {
                m_state = DONE;
                timer.Stop();
                m_vel = {0.0, 0.0};
            }

            break;
        
        case DONE:
            std::cout << "sad auto done" << "\n";
            m_ei.Kill();
            break;
    }
}

SadAuto::State SadAuto::GetCurState() {
    return m_state;
}

vec::Vector2D SadAuto::GetVelocity() {
    return m_vel;
}

std::string SadAuto::StateToString(State state) {
    switch (state) {
        case NOT_STARTED:
            return "NOT STARTED";
        case PLACING_UP:
            return "PLACING UP";
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