#include "Auto/EIAutoPath.h"
#include "Auto/AutoConstants.h"
#include <iostream>

EIAutoPath::EIAutoPath(ElevatorIntake::TargetState action, bool cone): m_action(action), m_cone(cone){
    std::cout << "ei auto path constructor called"<< std::endl;
    m_type = EI;
}

void EIAutoPath::Init(ElevatorIntake& ei){
    std::cout << "ei auto path init called"<< std::endl;
    m_EI = &ei;
    m_EI->SetCone(m_cone);
    std::cout << "ei auto path init finished"<< std::endl;
}

// void EIAutoPath::Periodic(){
//     m_EI->Periodic();
// }

void EIAutoPath::AutonomousPeriodic() {
    frc::SmartDashboard::PutNumber("completion", m_completion);
    frc::SmartDashboard::PutBoolean("started", m_started);
    if (m_started){
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
            break;
       }
    }
}

void EIAutoPath::Start(){
    std::cout << "ei auto path start called"<< std::endl;
    m_started = true;
    switch (m_action){
            case ElevatorIntake::TargetState::STOWED:
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
                m_EI->IntakeFromHPS();
                break;
            case ElevatorIntake::TargetState::GROUND:
                m_EI->IntakeFromGround();
                break;
        }
        std::cout << "ei auto path start finished"<< std::endl;
    }
