#include "IntakeConstants.h"
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Intake{
    public:
        Intake();

        enum State{
            DEPLOYING,
            DEPLOYED,
            STOWING,
            STOWED,
            STOPPED
        };

        void TeleopPeriodic();    
        void Stow();
        void DeployIntake(); // would be called intake for readability but can't bc class is Intake
        void DeployOuttake();
        void Kill();
        void ResetPID();

        // send lidar data
    private:
        void UpdatePose();
        double FFPIDCalculate();
        void UpdateTargetPose();
        void CalcVelTurnPos();
        double StepsToRad(double steps);

        bool dbg =true;
        bool m_outtaking = true;

        WPI_TalonFX m_wristMotor{IntakeConstants::WRIST_MOTOR_ID};
        WPI_TalonFX m_rollerMotor{IntakeConstants::ROLLER_MOTOR_ID};

        State m_state = STOWED;
        double m_kp = IntakeConstants::EXTEND_DEPLOY_P, m_ki = IntakeConstants::EXTEND_DEPLOY_I, 
               m_kd = IntakeConstants::EXTEND_DEPLOY_D, m_s = IntakeConstants::EXTEND_DEPLOY_S,
               m_g = IntakeConstants::EXTEND_DEPLOY_G, m_v = IntakeConstants::EXTEND_DEPLOY_V,
               m_a = IntakeConstants::EXTEND_DEPLOY_A;
        double m_setPt;
        double m_curPos, m_curVel, m_curAcc, m_targetPos, m_targetVel, m_targetAcc;
        double m_velTurnPos, m_totalErr = 0;
        //add caleb's lidar class when thats a thing
};