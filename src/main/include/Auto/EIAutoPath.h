#pragma once

#include "AutoPath.h"
#include "Elevator/ElevatorIntake.h"
#include "Elevator/Intake/Rollers.h"
#include "AutoConstants.h"

class EIAutoPath: public AutoPath {
    public:
        EIAutoPath(ElevatorIntake::TargetState action, bool cone);
        void Init(ElevatorIntake& ei, Rollers& r);
        void AutonomousPeriodic() override;
        void Start() override;
        std::string toString() override;
        void EnableDBG(bool dbg = true);
    
    private:
        ElevatorIntake::TargetState m_action;
        ElevatorIntake* m_EI;
        Rollers* m_rollers;
        
        bool m_cone;
        bool dbg = false;

};