#pragma once
#include "Elevator/Intake/Intake.h"
#include "elevator/BaseElevator.h"
#include "Lidar/LidarReader.h"

class ElevatorIntake{
    public:
        enum MechanismState{
            EXTENDING,
            EXTENDED,
            STOWING,
            STOWED,
            STOPPED
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

        //scoring stuff
        double m_yoff, m_zoff, m_scorAngle;
        
        //stuff fed to children 
        double m_targIntakeAng, m_targElevatorPos;

        Intake m_intake;
        LidarReader m_lidar;
        BaseElevator m_elevator;
};