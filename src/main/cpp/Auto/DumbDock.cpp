#include "Auto/DumbDock.h"

void DumbDock::Periodic(){
    m_curTime = Utils::GetCurTimeS();
    m_ei.TeleopPeriodic();
    m_rollers.Periodic();
    frc::SmartDashboard::PutNumber("m_vel x", m_vel.x());
    frc::SmartDashboard::PutNumber("m_vel y", m_vel.y());
    switch(m_state){
        case NOT_STARTED:
            frc::SmartDashboard::PutString("state", "not started");
            Start();
            break;
        case IN:
            frc::SmartDashboard::PutString("state", "in");
            if (m_curTime<IN_TIME+m_startTime){
                m_vel = {m_blue? -IN_SPEED:IN_SPEED, 0.0};
            } else {
                m_vel = {0.0, 0.0};
                m_ei.ScoreHigh();
                m_state = SCORE;
            }
            break;
        case SCORE:
            frc::SmartDashboard::PutString("state", "score");
            if (m_ei.IsDone()){
                m_rollers.Outtake();
                m_startTime = m_curTime;
                m_state = OUTTAKE;
            }
            break;
        case OUTTAKE:
            frc::SmartDashboard::PutString("state", "outtake");
            if (m_curTime > m_startTime + OUTTAKE_TIME ){
                m_rollers.Stop();
                m_ei.Stow();
                m_state = PREP4EXIT;
            }
            break;
        case PREP4EXIT:
            frc::SmartDashboard::PutString("state", "prep4exit");
            if (m_ei.IsDone()){
                m_startTime = m_curTime;
                m_vel = {m_blue? OUT_SPEED:-OUT_SPEED, 0.0};
                m_state = EXIT_COMMUNITY;
            }
            break;
        case EXIT_COMMUNITY:
            frc::SmartDashboard::PutString("state", "exit com");
            if (m_curTime > m_startTime + OUT_TIME){
                m_vel = {0.0, 0.0};
                m_state = WAIT;
                m_startTime = m_curTime;
            }
            break;
         case WAIT:
            frc::SmartDashboard::PutString("state", "wait");
            if (m_curTime > m_startTime + DONE_WAIT_TIME){
                m_state = DONE;
            }
            break;
        case DONE:
            frc::SmartDashboard::PutString("state", "done");
            break;
    }
}

void DumbDock::Start(){
    m_state = IN;
    m_startTime = m_curTime;
    m_ei.SetCone(!m_cube);
    m_rollers.SetCone(!m_cube);
}

double DumbDock::GetAngleVel(){return 0.0;}

vec::Vector2D DumbDock::GetVel(){return m_vel;}

bool DumbDock::CanDock(){return m_state == DONE;}

void DumbDock::SetSide(bool blue){m_blue = blue;}

void DumbDock::Reset(){m_state = NOT_STARTED;}


DumbDock::DumbDock(ElevatorIntake& e, Rollers& r):m_ei{e}, m_rollers{r}{}
