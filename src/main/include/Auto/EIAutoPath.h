#pragma once

#include "AutoPath.h"
#include "Elevator/ElevatorIntake.h"
#include "AutoConstants.h"

class EIAutoPath: public AutoPath {
    public:
        EIAutoPath(ElevatorIntake::TargetState action, bool cone);
        static void Init();
        static void Periodic();
        void AutonomousPeriodic() override;
        void Start() override;
    
    private:
        ElevatorIntake::TargetState m_action;
        static ElevatorIntake m_EI;

};