#pragma once
#include "Elevator/Intake/Intake.h"
#include <iostream>
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

        enum TargetState{
            STOWED,
            LOW,
            MID,
            HIGH,
            HP,
            GROUND
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
        // TargetState GetState();
        void IntakeFromGround();
        void IntakeFromHPS();
        void UpdateLidarData(LidarReader::LidarData& lidarData);
        void UpdateShuffleboard();
        void ManualPeriodic(double elevator, double intake);
        bool CanMoveFast() const;
        MovingState GetState();
        double GetWristCompletion();
        double GetElevatorCompletion();

    private:
        void DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo);
        IntakeElevatorConstants::GamePieceInfo GetGPI(bool cone);
        void Debug();
        void DebugScoring();
        void CalcIntakeDeployPos();

        bool dbg = false, dbg2= true;
        
        MechanismState m_state = MOVING;
        MovingState m_movingState = DONE;
        TargetState m_targState;
        bool m_cone, m_outtaking;

        bool m_rollers = false;
        double m_targIntakeAng = 0.0, m_targElevatorPos = 0.0;
        double m_startIntakeAng= 0.0, m_startElevatorPos = 0.0;

        bool m_useLidar = true;

        Elevator m_elevator{true, false};
        Intake m_intake{};

        IntakeElevatorConstants::GamePieceInfo coneinfo = IntakeElevatorConstants::coneScoreInfo;
        IntakeElevatorConstants::GamePieceInfo cubeinfo = IntakeElevatorConstants::cubeScoreInfo;

        // for debug
        IntakeElevatorConstants::GamePieceInfo& curGPInfo = coneinfo;
};