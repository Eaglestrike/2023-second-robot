#include "Drive/Auto/EIAutoPath.h"

EIAutoPath::EIAutoPath(ElevatorIntake::TargetState action, bool cone, double cue): m_action(action), AutoPath(cue){
    m_EI.SetCone(cone);
}

void EIAutoPath::Init(){
    m_EI.Init();
}

void EIAutoPath::Periodic(){
    m_EI.Periodic();
}

void EIAutoPath::AutonomousPeriodic() {
    if (m_started){
       switch (m_EI.GetState()){
        case ElevatorIntake::HALFSTOWING:
            m_completion = EIAutoConstants::HALFSTOW_PERCENT * m_EI.GetWristCompletion();
            break;
        case ElevatorIntake::ELEVATOR:
            m_completion = EIAutoConstants::ELEVATOR_PERCENT * m_EI.GetElevatorCompletion() + EIAutoConstants::HALFSTOW_PERCENT; 
            break;
        case ElevatorIntake::INTAKE:
            m_completion = EIAutoConstants::INTAKE_PERCENT * m_EI.GetElevatorCompletion() + EIAutoConstants::ELEVATOR_PERCENT + EIAutoConstants::HALFSTOW_PERCENT; 
            break;
       }
    }
}

void EIAutoPath::Start(){
    m_started = true;
    switch (m_action){
            case ElevatorIntake::TargetState::STOWED:
                m_EI.Stow();
                break;
            case ElevatorIntake::TargetState::LOW:
                m_EI.ScoreLow();
                break;
            case ElevatorIntake::TargetState::MID:
                m_EI.ScoreMid();
                break;
            case ElevatorIntake::TargetState::HIGH:
                m_EI.ScoreHigh();
                break;
            case ElevatorIntake::TargetState::HP:
                m_EI.IntakeFromHPS();
                break;
            case ElevatorIntake::TargetState::GROUND:
                m_EI.IntakeFromGround();
                break;
        }
    }
