#include "Auto/EIAutoPath.h"
#include "Auto/AutoConstants.h"
#include <iostream>

EIAutoPath::EIAutoPath(ElevatorIntake::TargetState action, bool cone): AutoPath(EIAutoConstants::COMPLETION_TOLERANCE), m_action(action), m_cone(cone){
    if (dbg) std::cout << "ei auto path constructor called"<< std::endl;
    m_type = EI;
}

void EIAutoPath::Init(ElevatorIntake& ei, Rollers& r){
    if (dbg) std::cout << "ei auto path init called"<< std::endl;
    m_EI = &ei;
    m_rollers = &r;
    m_rollers->SetCone(m_cone);
    m_EI->SetCone(m_cone);
    if (dbg) std::cout << "ei auto path init finished"<< std::endl;
}

std::string EIAutoPath::toString(){
    std::string s = "";
    switch(m_action){
        case ElevatorIntake::STOWED:
            s += "stow";
            break;
        case ElevatorIntake::LOW:
         s += "low";
            break;
        case ElevatorIntake::MID:
         s += "mid";
            break;
        case ElevatorIntake::HIGH:
         s += "high";
            break;
        case ElevatorIntake::HP:
         s += "hp";
            break;
        case ElevatorIntake::GROUND:
         s += "grnd";
            break;
    }

    if (m_cone){
        s+= " cone";
    } else s += " cube";
    return s;
}

void EIAutoPath::AutonomousPeriodic() {
    if (dbg){
        frc::SmartDashboard::PutNumber("completion", m_completion);
        frc::SmartDashboard::PutBoolean("started", m_started);
    }
    
    if (m_started){
        if (dbg){
            frc::SmartDashboard::PutNumber("w comp", m_EI->GetWristCompletion());
        frc::SmartDashboard::PutNumber("e comp", m_EI->GetElevatorCompletion());
        }
        m_EI->TeleopPeriodic();
       switch (m_EI->GetState()){
        case ElevatorIntake::HALFSTOWING:
            m_completion = EIAutoConstants::HALFSTOW_PERCENT * m_EI->GetWristCompletion();
            break;
        case ElevatorIntake::ELEVATOR:
            m_completion = EIAutoConstants::ELEVATOR_PERCENT * m_EI->GetElevatorCompletion() + EIAutoConstants::HALFSTOW_PERCENT; 
            break;
        case ElevatorIntake::INTAKE:
            m_completion = EIAutoConstants::INTAKE_PERCENT * m_EI->GetWristCompletion() + EIAutoConstants::ELEVATOR_PERCENT + EIAutoConstants::HALFSTOW_PERCENT; 
        case ElevatorIntake::DONE:
            if (m_action == ElevatorIntake::TargetState::LOW || m_action == ElevatorIntake::TargetState::MID || ElevatorIntake::TargetState::HIGH){
                if (m_rollers->GetState() != Rollers::OUTTAKE)
                    m_rollers->Outtake();
                else if (m_rollers->GetState() == Rollers::OUTTAKE && !m_rollers->HasGamePiece()) {
                    m_rollers->Stop();
                    m_completion = 1.0;
                    m_started = false;
                }
            } else if (m_rollers->HasGamePiece()){
                m_completion = 1.0;
                m_started = false;
            }
            break;
       }
    }
}

void EIAutoPath::Start(){
    if (dbg) std::cout << "ei auto path start called"<< std::endl;
    m_started = true;
    switch (m_action){
            case ElevatorIntake::TargetState::STOWED:
                if (!m_rollers->HasGamePiece()) m_rollers->Stop();
                m_EI->Stow();
                break;
            case ElevatorIntake::TargetState::LOW:
                m_EI->ScoreLow();
                break;
            case ElevatorIntake::TargetState::MID:
                m_EI->ScoreMid();
                break;
            case ElevatorIntake::TargetState::HIGH:
                m_EI->ScoreHigh();
                break;
            case ElevatorIntake::TargetState::HP:
                m_rollers->Intake();
                m_EI->IntakeFromHPS();
                break;
            case ElevatorIntake::TargetState::GROUND:
                m_rollers->Intake();
                m_EI->IntakeFromGround();
                break;
    }
    if (dbg) std::cout << "ei auto path start finished"<< std::endl;
}

void EIAutoPath::EnableDBG(bool d){
    dbg = d;
}
