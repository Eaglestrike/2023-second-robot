#pragma once
#include "Elevator/Intake/Intake.h"
#include "BaseElevator/Elevator.h"
#include "Lidar/LidarReader.h"

class ElevatorIntake : public Mechanism{
    public:
        enum MechanismState{
            MANUAL,
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

        ElevatorIntake(std::string name, bool enabled, bool shuffleboard);
        void ToggleRoller(bool outtaking);
        void DeployElevatorIntake(double elevatorLength, double intakeDeg);
        void Kill();

        void Stow();
        void ScoreHigh();
        void ScoreMid();
        void ScoreLow();

        void IntakeFromGround();
        void IntakeFlange();
        void IntakeFromHPS();

        bool CanMoveFast() const;
        
        void SetManualVolts(double elevator, double intake);
        void SetCone(bool cone);

    private:
        void CoreInit() override;
        void CorePeriodic() override;
        void CoreTeleopPeriodic() override;

        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;
        void CoreShuffleboardUpdate() override;

        void DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo);
        IntakeElevatorConstants::GamePieceInfo GetGPI(bool cone);
        void CalcIntakeDeployPos();
        
        MechanismState m_state = MOVING;
        MovingState m_movingState = DONE;
        TargetState m_targState;
        bool m_cone;

        double m_targIntakeAng, m_targElevatorPos;
        double m_manualIntake, m_manualElevator;

        bool m_useLidar = true;

        Elevator m_elevator{true, true};
        Intake m_intake;

        IntakeElevatorConstants::GamePieceInfo coneinfo = IntakeElevatorConstants::coneScoreInfo;
        IntakeElevatorConstants::GamePieceInfo cubeinfo = IntakeElevatorConstants::cubeScoreInfo;

        // for debug
        IntakeElevatorConstants::GamePieceInfo& curGPInfo = coneinfo;
};