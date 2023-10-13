#pragma once

#include "Elevator/ElevatorIntakeConstants.h"
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>

class Intake{
    public:
        Intake();

        enum WristState{
            STOWING,
            DEPLOYING,
            DEPLOYED,
            STOWED,
            HALFSTOWING,
            STOPPED,
            HALFSTOWED,
        };

        void ManualPeriodic(double wristVolts, double rollerVolts);
        void TeleopPeriodic();    
        void Stow();
        void HalfStow();
        void DeployIntake(bool cone); 
        void DeployOuttake(bool cone);
        void DeployNoRollers();
        void ChangeDeployPos(double newPos);
        void ChangeRollerVoltage(double newVolotage); //pos should be in radians, w 0 as extended and parallel to ground
        void Kill();
        WristState GetState();
        double GetPos();
    private:
        void UpdatePose();
        void UpdateTargetPose();
        double FFPIDCalculate();
        void CalcSpeedDecreasePos();
        void SetSetpoint(double setpt);
        bool AtSetpoint();
        void ResetPID();
        double StepsToRad(double steps);

        void debugTargPose();
        void debugCurPose();
        void debugPutVoltage();

        bool dbg =true;

        rev::CANSparkMax m_wristMotor{IntakeConstants::WRIST_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
        WPI_TalonFX m_rollerMotor{IntakeConstants::ROLLER_MOTOR_ID};
        frc::DutyCycleEncoder m_wristEncoder{IntakeConstants::WRIST_ENCODER_CAN_ID};

        WristState m_state = STOWED;
        double m_kp = IntakeConstants::EXTEND_DEPLOY_P, m_ki = IntakeConstants::EXTEND_DEPLOY_I, 
               m_kd = IntakeConstants::EXTEND_DEPLOY_D, m_s = IntakeConstants::EXTEND_DEPLOY_S,
               m_g = IntakeConstants::EXTEND_DEPLOY_G, m_v = IntakeConstants::EXTEND_DEPLOY_V,
               m_a = IntakeConstants::EXTEND_DEPLOY_A;
        
        double m_setPt; 
        double m_curPos, m_curVel, m_curAcc; // cur pose
        double m_targetPos, m_targetVel, m_targetAcc; // motion profile 
        double m_speedDecreasePos, // pos in motion profile where start decelerating
               m_totalErr = 0; // integral of position error for PID

        double m_rollerVolts;
        double m_customDeployPos =-1, m_customRollerVolts = -1;

        frc2::PIDController m_stowedPIDcontroller{IntakeConstants::STOW_P,IntakeConstants::STOW_I,IntakeConstants::STOW_D};
        //add caleb's lidar class when thats a thing
};