#include "IntakeConstants.h"
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>

class Intake{
    public:
        Intake();

        enum WristState{
            DEPLOYING,
            DEPLOYED,
            STOWING,
            STOWED,
            STOPPED
        };

        // enum IntakingState{
        //     INTAKING,
        //     OUTTAKING
        // };

        void TeleopPeriodic();    
        void Stow();
        void DeployIntake(); // would be called intake for readability but can't bc class is Intake
        void DeployOuttake();
        void ChangeWristPos(double newPos); //pos should be in radians, w 0 as extended and parallel to ground
        void ChangeRollerVoltage(double newVolotage, bool out); //pos should be in radians, w 0 as extended and parallel to ground
        void Kill();
        // send lidar data
    private:
        void UpdatePose();
        void UpdateTargetPose();
        double FFPIDCalculate();
        void CalcVelTurnPos();
        void SetSetpoint(double setpt);
        bool AtSetpoint();
        void ResetPID();
        double StepsToRad(double steps);

        bool dbg =true;
        // IntakingState m_intakingState;

        WPI_TalonFX m_wristMotor{IntakeConstants::WRIST_MOTOR_ID};
        WPI_TalonFX m_rollerMotor{IntakeConstants::ROLLER_MOTOR_ID};
        frc::DutyCycleEncoder m_wristEncoder{IntakeConstants::WRIST_ENCODER_PIN};

        WristState m_state = STOWED;
        double m_kp = IntakeConstants::EXTEND_DEPLOY_P, m_ki = IntakeConstants::EXTEND_DEPLOY_I, 
               m_kd = IntakeConstants::EXTEND_DEPLOY_D, m_s = IntakeConstants::EXTEND_DEPLOY_S,
               m_g = IntakeConstants::EXTEND_DEPLOY_G, m_v = IntakeConstants::EXTEND_DEPLOY_V,
               m_a = IntakeConstants::EXTEND_DEPLOY_A;
        
        double m_setPt; 
        double m_curPos, m_curVel, m_curAcc; // cur pose
        double m_targetPos, m_targetVel, m_targetAcc; // motion profile 
        double m_velTurnPos, // pos in motion profile where start decelerating
               m_totalErr = 0; // integral of position error for PID

        double m_rollerVolts;
        //add caleb's lidar class when thats a thing
};