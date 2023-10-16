#pragma once
#include "Elevator/Intake/Intake.h"
#include "BaseElevator/Elevator.h"
#include "Lidar/LidarReader.h"

class ElevatorIntake{
    public:
        enum MechanismState{
            MOVING,
            STOPPED
        };

        enum MovingState{
            HALFSTOWING,
            ELEVATOR,
            INTAKE,
            DONE
        };

        ElevatorIntake();
        void Periodic();
        void TeleopPeriodic();
        void Kill();
        void DeployElevatorIntake(double elevatorLength, double intakeDeg);
        void Stow();
        void ScoreHigh(bool cone);
        void ScoreMid(bool cone);
        void ScoreLow(bool cone);
        void IntakeFromGround(bool cone);
        void IntakeFromHPS(bool cone);
    private:
        void DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo, bool cone, bool outtaking);
        IntakeElevatorConstants::GamePieceInfo GetGPI(bool cone);
        void Debug();
        void DebugScoring();
        void CalcIntakeDeployPos();

        bool dbg = false;
        
        MechanismState m_state = MOVING;
        MovingState m_movingState = DONE;
        bool m_outtaking, m_cone;

        bool m_stowing;
        double m_targIntakeAng, m_targElevatorPos;

        Elevator m_elevator{true, false};
        // tbh might move lidar out of this 
        LidarReader m_lidar{true, false};
        Intake m_intake{m_lidar};
};