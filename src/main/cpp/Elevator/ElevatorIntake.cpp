#include "Elevator/ElevatorIntake.h"

ElevatorIntake::ElevatorIntake(){
    m_elevator.Init();
    frc::SmartDashboard::PutNumber("elevator len", 0.0);
    frc::SmartDashboard::PutNumber("wrist angle", 0.0);
    frc::SmartDashboard::PutBoolean("deploy", false);
    frc::SmartDashboard::PutBoolean("stow", false);
    frc::SmartDashboard::PutBoolean("outtake", false);
    frc::SmartDashboard::PutBoolean("cone", false);
}

// void ElevatorIntake::CalcIntakeAngle(){}

void ElevatorIntake::DeployElevatorIntake(double elevatorLength, double intakeAng){
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = elevatorLength;
    m_targIntakeAng = intakeAng;
    m_stowing = false;
}

void ElevatorIntake::Stow(){
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = ElevatorConstants::STOWED_HEIGHT;
    m_targIntakeAng =  IntakeConstants::STOWED_POS;
    m_stowing = true;
}

void ElevatorIntake::dbg(){
    m_targElevatorPos = frc::SmartDashboard::GetNumber("elevator len", 0.0);
    m_targIntakeAng = frc::SmartDashboard::GetNumber("wrist angle", 0.0);
    if(frc::SmartDashboard::GetBoolean("deploy", false)){
        DeployElevatorIntake(m_targElevatorPos, m_targIntakeAng);
        frc::SmartDashboard::PutBoolean("deploy", false);
    }

    if(frc::SmartDashboard::GetBoolean("stow", false)){
        Stow();
        frc::SmartDashboard::PutBoolean("stow", false);
    }
    m_outtaking = frc::SmartDashboard::GetBoolean("outtake", false);
    m_cone = frc::SmartDashboard::GetBoolean("cone", false);
    frc::SmartDashboard::PutNumber("elevator acc pos", m_elevator.getElevatorHeight());
    frc::SmartDashboard::PutNumber("intake acc angle", m_intake.GetPos());

    frc::SmartDashboard::PutNumber("moving state", m_movingState);
    frc::SmartDashboard::PutNumber("mechanism state", m_state);

    frc::SmartDashboard::PutNumber("intake targ state", m_intake.GetTargetState());
    frc::SmartDashboard::PutNumber("intake state", m_intake.GetState());
    frc::SmartDashboard::PutString("elevator state", m_elevator.getStateString());
    frc::SmartDashboard::PutString("elevator target", m_elevator.getTargetString());
}

void ElevatorIntake::CalcIntakeDeployPos(){}

void ElevatorIntake::TeleopPeriodic(){
    dbg();
    m_intake.TeleopPeriodic();
    m_elevator.Periodic();
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
                        if(m_stowing)
                            m_elevator.Stow();
                        else
                            m_elevator.ExtendToCustomPos(m_targElevatorPos);
                        m_movingState = ELEVATOR;
                    }
                    break;
                case ELEVATOR:
                    if (m_elevator.getState() == Elevator::HOLDING_POS){    
                        if (m_stowing){
                            m_intake.Stow();
                        } else {
                            m_intake.ChangeDeployPos(m_targIntakeAng);
                            if (!m_outtaking) m_intake.DeployIntake(m_cone);
                            else m_intake.DeployOuttake(m_cone);
                        }
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

