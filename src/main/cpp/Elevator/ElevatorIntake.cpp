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
    
}

void ElevatorIntake::dbg(){

    m_targElevatorPos = frc::SmartDashboard::GetNumber("elevator len", 0.0);
    m_targIntakeAng = frc::SmartDashboard::GetNumber("wrist angle", 0.0);
    if(frc::SmartDashboard::GetBoolean("deploy", false)){
        m_state = EXTENDING;
        frc::SmartDashboard::PutBoolean("deploy", false);
    }

    m_outtaking = frc::SmartDashboard::GetBoolean("outtake", false));
    m_cone = frc::SmartDashboard::GetBoolean("cone", false);
    
}

void ElevatorIntake::CalcIntakeDeployPos(){}

void ElevatorIntake::TeleopPeriodic(){
    m_intake.TeleopPeriodic();
    m_elevator.TeleopPeriodic();
    switch(m_state){
        case EXTENDING:
            if (m_intake.GetPos() < IntakeElevatorConstants::INTAKE_UPRIGHT_ANGLE){
                if (m_intake.GetState() != Intake::STOWING){
                    m_intake.ChangeDeployPos(IntakeElevatorConstants::INTAKE_UPRIGHT_ANGLE);
                    m_intake.DeployIntake(false);
                }
            } else {
                if (m_elevator.GetState() != BaseElevator::EXTENDING){
                    m_elevator.ChangeDeployPos(m_targElevatorPos);
                    m_elevator.Deploy();
                } else if (m_elevator.GetState() == BaseElevator::EXTENDED){
                    if (m_intake.GetState() != EXTENDING){
                        m_intake.ChangeDeployPos(m_targIntakeAng);
                        m_intake.DeployIntake();
                    }
                }
            }

            // else if (m_elevator.GetPos() >= m_intakeDeployPos && m_intake.GetState() == Intake::STOWED){
            //     CalcIntakeAngle(); // after calculating using how far its gone
            //     m_intake.ChangeDeployPos(m_targIntakeAng);
            //     if(!m_outtaking) m_intake.DeployIntake(m_cone);
            //     else m_intake.DeployOuttake(m_cone);
            // } else if (m_elevator.GetState() == BaseElevator::EXTENDED
            //     && m_intake.GetState() == Intake::DEPLOYED)
            //     m_state = EXTENDED;

            break;
        case STOWING:
            if(m_intake.GetPos() >= m_elevatorStowPos){
                m_elevator.Stow();
            }
            if (m_elevator.GetState() == BaseElevator::STOWED
                && m_intake.GetState() == Intake::STOWED)
                m_state = STOWED;
            break;
    }
}

void ElevatorIntake::CalcToCustomPose(double yoff, double zoff, IntakeElevatorConstants::IdealScoreInfo scoreInfo){
    m_yoff = yoff; m_zoff = zoff;
    m_targIntakeAng = IntakeElevatorConstants::ELEVATOR_ANGLE - asin((sin(IntakeElevatorConstants::ELEVATOR_ANGLE)*(yoff-IntakeElevatorConstants::INTAKE_BAR_LENGTH)- cos(IntakeElevatorConstants::ELEVATOR_ANGLE)*zoff)/scoreInfo.INTAKE_LEN);
    m_targElevatorPos = (zoff-scoreInfo.INTAKE_LEN*sin(m_targIntakeAng))/sin(IntakeElevatorConstants::ELEVATOR_ANGLE);
    m_scorAngle = (M_PI/2 - m_targIntakeAng - scoreInfo.ANGLE_W_INTAKE);
}

void ElevatorIntake::CalcToCustomPose(double yoff, double scoringAngle, IntakeElevatorConstants::IdealScoreInfo scoreInfo){

}

void ElevatorIntake::CalcToCustomPose(double zoff, double scoringAngle, IntakeElevatorConstants::IdealScoreInfo scoreInfo){

}


/*
assumes the robot is already faced perfectly towards wherever scoring to
meaning xoffset should be 0.

yoff is distance from center of robot to the place you want to the intake to be
*/
void ElevatorIntake::IntakeFromCustomPos(double yoff, double targHeight, bool cone){
    m_yoff = yoff; m_zoff = targHeight;
    // Calc
    CalcIntakeDeployPos();
    m_outtaking = false;
    m_elevator.ExtendToCustomPos(m_targElevatorPos);
    m_cone = cone;
}


// yoff and target hight refer to the coordinate that you want to SCORE at
void ElevatorIntake::OuttakeToCustomPos(double yoff, double targHeight){
    //use lidar data also for cone and cubes 
    m_yoff = yoff; m_zoff = targHeight;
    // CalcElevatorPosIntakeAng();
    CalcIntakeDeployPos();
    m_outtaking = true;
    m_elevator.ExtendToCustomPos(m_targElevatorPos);
    m_cone = m_lidar.hasCone();
}

void ElevatorIntake::Stow(){
    m_state = STOWING;
    m_intake.Stow();
}

void ElevatorIntake::Kill(){
    m_state = STOPPED;
}

