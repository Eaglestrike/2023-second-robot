#pragma once

#include <ctre/Phoenix.h>

#include <rev/CANSparkMax.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>

#include "Elevator/ElevatorIntakeConstants.h"
#include "Elevator/Lidar/LidarReader.h"

#include "Util/Mechanism.h"
#include "Util/Utils.h"


class Intake : public Mechanism{
    public:
        Intake(std::string name, bool enabled, bool shuffleboard);

        enum MechState{
            MOVING,
            AT_TARGET,
            STOPPED,
            MANUAL
        };

        enum TargetState{
            STOWED,
            DEPLOYED,
            HALFSTOWED
        };

        void Stow();
        void HalfStow();

        void Deploy(); 
        void ChangeDeployPos(double newPos); //pos should be in radians, w 0 as extended and parallel to ground

        void Kill();

        // for debugging
        MechState GetState();
        TargetState GetTargetState();
        double GetPos();

        void setManualVolts(double wristVolts);

    private:
        void CoreTeleopPeriodic() override; 
        void CorePeriodic() override;

        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;
        void CoreShuffleboardUpdate() override;

        void UpdatePose();
        void UpdateTargetPose();
        double FFPIDCalculate();

        void CalcSpeedDecreasePos();

        void SetSetpoint(double setpt);
        bool AtSetpoint();
        void ResetPID();

        rev::CANSparkMax m_wristMotor{IntakeConstants::WRIST_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        frc::DutyCycleEncoder m_wristEncoder{IntakeConstants::WRIST_ENCODER_CAN_ID};

        TargetState m_targState = TargetState::STOWED;
        MechState m_state = MechState::AT_TARGET;

        double m_kp = IntakeConstants::EXTEND_DEPLOY_P, m_ki = IntakeConstants::EXTEND_DEPLOY_I, 
               m_kd = IntakeConstants::EXTEND_DEPLOY_D, m_s = IntakeConstants::EXTEND_DEPLOY_S,
               m_g = IntakeConstants::EXTEND_DEPLOY_G, m_v = IntakeConstants::EXTEND_DEPLOY_V,
               m_a = IntakeConstants::EXTEND_DEPLOY_A;
        
        double m_setPt = IntakeConstants::STOWED_POS; 

        double m_curPos, m_curVel, m_curAcc; // cur pose

        double m_targetPos = m_setPt, m_targetVel =0 , m_targetAcc = 0; // motion profile 

        double m_speedDecreasePos, // pos in motion profile where start decelerating
               m_totalErr = 0; // integral of position error for PID
               
        double m_customDeployPos = -1;

        double m_manualVolts = 0.0;
};