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
        void TeleopPeriodic();
        void Kill();
        void DeployElevatorIntake(double elevatorLength, double intakeDeg);
        // void IntakeFromCustomPos(double yoff, double targHeight, bool cone);
        // void OuttakeToCustomPos(double yoff, double targHeight);
        void Stow();

    private:
        void dbg();
        void CalcIntakeDeployPos();
        // void CalcToCustomPose(double yoff, double zoff, IntakeElevatorConstants::IdealScoreInfo scoreInfo);
        // void CalcToCustomPose(double yoff, double scoringAngle, IntakeElevatorConstants::IdealScoreInfo scoreInfo);
        // void CalcToCustomPose(double zoff, double scoringAngle, IntakeElevatorConstants::IdealScoreInfo scoreInfo);
        // void CalcIntakeAngle();
        
        MechanismState m_state;
        bool m_outtaking, m_cone;

        MovingState m_movingState;

        //scoring stuff
        // not used
        // double m_yoff, m_zoff, m_scorAngle;
        
        //stuff fed to children 
        double m_targIntakeAng, m_targElevatorPos;

        Intake m_intake;
        Elevator m_elevator{true, false};

        // tbh might move lidar out of this 
        // LidarReader m_lidar;
};