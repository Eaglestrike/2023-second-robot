#pragma once

#include "AutoPaths.h"
#include "Elevator/ElevatorIntake.h"

class EIAutoPath: public AutoPaths {
    public:
        EIAutoPath(ElevatorIntake::TargetState action);
        void Periodic() override;
        void AutonomousPeriodic() override;
        void Init() override;
        void Start() override;
    
    private:
        ElevatorIntake::TargetState m_action;
        ElevatorIntake m_EI;

};