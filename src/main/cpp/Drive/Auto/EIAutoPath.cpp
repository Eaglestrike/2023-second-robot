#include "Drive/Auto/EIAutoPath.h"

EIAutoPath::EIAutoPath(ElevatorIntake::TargetState action): m_action(action){}

void EIAutoPath::Init(){
    m_EI.Init();
}

void EIAutoPath::Periodic(){
    m_EI.Periodic();
}

void EIAutoPath::AutonomousPeriodic() {
    if (m_started){
        switch (m_action){
            case ElevatorIntake::TargetState::STOWED:
                break;
            case ElevatorIntake::TargetState::LOW:
                break;
            case ElevatorIntake::TargetState::MID:
                break;
            case ElevatorIntake::TargetState::HIGH:
                break;
            case ElevatorIntake::TargetState::HP:
                break;
            case ElevatorIntake::TargetState::GROUND:
                break;
        }
    }
}