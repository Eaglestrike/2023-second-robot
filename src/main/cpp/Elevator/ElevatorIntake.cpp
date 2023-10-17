#include "Elevator/ElevatorIntake.h"

ElevatorIntake::ElevatorIntake(){
    // m_intake = Intake{m_lidar};
    if (dbg){
        frc::SmartDashboard::PutNumber("elevator len", 0.0);
        frc::SmartDashboard::PutNumber("wrist angle", 0.0);
        frc::SmartDashboard::PutBoolean("deploy", false);
        frc::SmartDashboard::PutBoolean("stow", false);
        frc::SmartDashboard::PutBoolean("outtake", false);
        frc::SmartDashboard::PutBoolean("cone", false);
    }
}

void ElevatorIntake::Init() {
    m_elevator.Init();
}

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

void ElevatorIntake::Debug(){
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

void ElevatorIntake::DebugScoring(){
    frc::SmartDashboard::PutBoolean("outtake", m_outtaking);
    frc::SmartDashboard::PutBoolean("cone", m_cone);
    frc::SmartDashboard::PutNumber("elevator acc pos", m_elevator.getElevatorHeight());
    frc::SmartDashboard::PutNumber("intake acc angle", m_intake.GetPos());
    frc::SmartDashboard::PutNumber("elevator targ pos", m_targElevatorPos);
    frc::SmartDashboard::PutNumber("intake targ angle", m_targIntakeAng);
}

void ElevatorIntake::Periodic(){
    m_intake.Periodic();
    m_elevator.Periodic();
}

void ElevatorIntake::ToggleRoller(bool outtaking){
    if (m_rollers){
        m_intake.StopRollers();
        m_rollers = false;
    } else {
        m_intake.StartRollers(outtaking, m_cone); 
        m_rollers = true;
    }
}

void ElevatorIntake::TeleopPeriodic(){
   if (dbg) Debug();
   DebugScoring();

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
                            m_intake.DeployNoRollers();
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

void ElevatorIntake::ScoreHigh(bool cone){
    DeployElevatorIntake(GetGPI(cone).SCORE_HIGH, cone, true);
}

void ElevatorIntake::ScoreMid(bool cone){
    DeployElevatorIntake(GetGPI(cone).SCORE_MID, cone, true);
}
void ElevatorIntake::ScoreLow(bool cone){
   DeployElevatorIntake(GetGPI(cone).SCORE_LOW, cone, true);
}

void ElevatorIntake::IntakeFromGround(bool cone){
   DeployElevatorIntake(GetGPI(cone).GROUND_INTAKE, cone, false);
}

void ElevatorIntake::IntakeFromHPS(bool cone){
   DeployElevatorIntake(GetGPI(cone).HP_INTAKE, cone, false);
}

IntakeElevatorConstants::GamePieceInfo ElevatorIntake::GetGPI(bool cone){
   if (cone) return IntakeElevatorConstants::coneScoreInfo;
   return IntakeElevatorConstants::cubeScoreInfo;
}

void ElevatorIntake::DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo, bool cone, bool outtaking){
    m_cone = cone;
    m_outtaking = outtaking;
    DeployElevatorIntake(scoreInfo.ELEVATOR_LENG, scoreInfo.INTAKE_ANGLE);
}
