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
        void Init();
        void Periodic();
        void TeleopPeriodic();
        void Kill();
        void ToggleRoller(bool outtaking);
        void DeployElevatorIntake(double elevatorLength, double intakeDeg);
        void Stow();
        void SetCone(bool cone);
        void ScoreHigh();
        void ScoreMid();
        void ScoreLow();
        void IntakeFromGround();
        void IntakeFromHPS();
        void ConeCubeManual();
        void ConeCubeLidar();
    private:
        void DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo);
        IntakeElevatorConstants::GamePieceInfo GetGPI(bool cone);
        void Debug();
        void DebugScoring();
        void CalcIntakeDeployPos();

        bool dbg = false;
        
        MechanismState m_state = MOVING;
        MovingState m_movingState = DONE;
        bool m_cone;

        bool m_stowing;
        bool m_rollers = false;
        double m_targIntakeAng, m_targElevatorPos;

        bool m_useLidar = true;

        Elevator m_elevator{true, false};
        // tbh might move lidar out of this 
        LidarReader m_lidar{true, false};
        Intake m_intake;

        IntakeElevatorConstants::GamePieceInfo coneinfo = IntakeElevatorConstants::coneScoreInfo;
        IntakeElevatorConstants::GamePieceInfo cubeinfo = IntakeElevatorConstants::cubeScoreInfo;

        // for debug
        IntakeElevatorConstants::GamePieceInfo& curGPInfo = coneinfo;
};