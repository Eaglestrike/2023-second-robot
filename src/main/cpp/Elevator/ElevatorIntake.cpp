#include "Elevator/ElevatorIntake.h"

ElevatorIntake::ElevatorIntake(){
    frc::SmartDashboard::PutNumber("elevator len", 0.0);
    frc::SmartDashboard::PutNumber("wrist angle", 0.0);
    frc::SmartDashboard::PutBoolean("deploy", false);
    frc::SmartDashboard::PutBoolean("outtake", false);
    frc::SmartDashboard::PutBoolean("cone", false);
}

// void ElevatorIntake::CalcIntakeAngle(){}

void ElevatorIntake::DeployElevatorIntake(double elevatorLength, double intakeAng){
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = elevatorLength;
    m_targIntakeAng = intakeAng;
}

void ElevatorIntake::Stow(){
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = ElevatorConstants::STOWED_HEIGHT;
    m_targIntakeAng =  IntakeConstants::STOWED_POS;
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
    frc::SmartDashboard::PutNumber("elevator acc pos", m_elevator.GetPos());
    frc::SmartDashboard::PutNumber("intake acc angle", m_intake.GetPos());
}

void ElevatorIntake::CalcIntakeDeployPos(){}

void ElevatorIntake::TeleopPeriodic(){
    dbg();
    m_intake.TeleopPeriodic();
    m_elevator.TeleopPeriodic();
    switch(m_state){
        case STOPPED:
            m_elevator.Disable();
            m_intake.Kill();
            break;
        case MOVING:
            switch(m_movingState){
                case HALFSTOWING:
                    if (m_intake.GetTargetState() != Intake::HALFSTOWED)
                        m_intake.HalfStow();
                    else if (m_intake.GetState() == Intake::AT_TARGET){
                        m_elevator.ExtendToCustomPos(m_targElevatorPos);
                        m_movingState = ELEVATOR;
                    }
                    break;
                case ELEVATOR:
                    if (m_elevator.getState() == Elevator::HOLDING_POS){
                       m_intake.ChangeDeployPos(m_targIntakeAng);
                        if (!m_outtaking) m_intake.DeployIntake(m_cone);
                        else m_intake.DeployOuttake(m_cone);
                        m_movingState = INTAKE;
                    }
                    break;
                case INTAKE:
                    if (m_intake.GetState() == Intake::AT_TARGET)
                        m_movingState = DONE;
                    break;
                case DONE:
                    break;
            }
            break;
    }
}



void ElevatorIntake::Kill(){
    m_state = STOPPED;
}

