#include "Elevator/ElevatorIntake.h"

ElevatorIntake::ElevatorIntake(){
    frc::SmartDashboard::PutNumber("elevator len", 0.0);
    frc::SmartDashboard::PutNumber("wrist angle", 0.0);
    frc::SmartDashboard::PutBoolean("deploy", false);
    frc::SmartDashboard::PutBoolean("outtake", false);
    frc::SmartDashboard::PutBoolean("cone", false);
}

// void ElevatorIntake::CalcIntakeAngle(){}

void ElevatorIntake::DeployElevatorIntake(double elevatorLength, double intakeDeg){
    m_state = EXTENDING;
}

void ElevatorIntake::dbg(){
    m_targElevatorPos = frc::SmartDashboard::GetNumber("elevator len", 0.0);
    m_targIntakeAng = frc::SmartDashboard::GetNumber("wrist angle", 0.0);
    if(frc::SmartDashboard::GetBoolean("deploy", false)){
        DeployElevatorIntake(m_targElevatorPos, m_targIntakeAng);
        frc::SmartDashboard::PutBoolean("deploy", false);
    }
    m_outtaking = frc::SmartDashboard::GetBoolean("outtake", false);
    m_cone = frc::SmartDashboard::GetBoolean("cone", false);
    
}

void ElevatorIntake::CalcIntakeDeployPos(){}

void ElevatorIntake::TeleopPeriodic(){
    dbg();
    m_intake.TeleopPeriodic();
    m_elevator.TeleopPeriodic();
    switch(m_state){
        case EXTENDING:
            if (m_intake.GetState() == Intake::STOWED){
                m_intake.HalfStow();
            } else if (m_intake.GetState() == Intake::HALFSTOWED){
                if (m_elevator.GetState() != BaseElevator::EXTENDING){
                    m_elevator.ExtendToCustomPos(m_targElevatorPos);
                } else if (m_elevator.GetState() == BaseElevator::EXTENDED){
                    m_intake.ChangeDeployPos(m_targIntakeAng);
                    if (!m_outtaking) m_intake.DeployIntake(m_cone);
                    else m_intake.DeployOuttake(m_cone);
                }
            } else if (m_intake.GetState() == EXTENDED && m_elevator.GetState() == BaseElevator::EXTENDED)
                m_state = EXTENDED;
            break;
        case STOWING:
            if (m_intake.GetState() == EXTENDED){
                m_intake.HalfStow();
            } else if (m_intake.GetState() == Intake::HALFSTOWED){
                m_elevator.Stow();
                if(m_elevator.GetState() == STOWED)
                    m_intake.Stow();
            } else if (m_intake.GetState() == STOWED && m_elevator.GetState() == STOWED)
                m_state = STOWED;
            // if (m_elevator.GetState() == BaseElevator::STOWED
            //     && m_intake.GetState() == Intake::STOWED)
            //     m_state = STOWED;
            break;
    }
}

void ElevatorIntake::Stow(){
    m_state = STOWING;
}

void ElevatorIntake::Kill(){
    m_state = STOPPED;
}

