#pragma once

#include "AutoPath.h"
#include "Elevator/ElevatorIntake.h"
#include "AutoConstants.h"

class EIAutoPath: public AutoPath {
    public:
        EIAutoPath(ElevatorIntake::TargetState action, bool cone);
        void Init(ElevatorIntake& ei);
        // static void Periodic();
        void AutonomousPeriodic() override;
        void Start() override;
    
    private:
        ElevatorIntake::TargetState m_action;
        ElevatorIntake* m_EI;
        bool m_cone;

};