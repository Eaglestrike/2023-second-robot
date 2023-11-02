#include "Elevator/ElevatorIntake.h"

ElevatorIntake::ElevatorIntake(){
    if (dbg){
        frc::SmartDashboard::PutNumber("elevator len", 0.0);
        frc::SmartDashboard::PutNumber("wrist angle", 0.0);
        frc::SmartDashboard::PutBoolean("deploy", false);
        frc::SmartDashboard::PutBoolean("stow", false);
        frc::SmartDashboard::PutBoolean("outtake", false);
        frc::SmartDashboard::PutBoolean("cone", false);

        frc::SmartDashboard::PutNumber("low e", curGPInfo.SCORE_LOW.ELEVATOR_LENG);
        frc::SmartDashboard::PutNumber("low w", curGPInfo.SCORE_LOW.INTAKE_ANGLE);
        frc::SmartDashboard::PutNumber("mid e", curGPInfo.SCORE_MID.ELEVATOR_LENG);
        frc::SmartDashboard::PutNumber("mid w", curGPInfo.SCORE_MID.INTAKE_ANGLE);
        frc::SmartDashboard::PutNumber("high e", curGPInfo.SCORE_HIGH.ELEVATOR_LENG);
        frc::SmartDashboard::PutNumber("high w", curGPInfo.SCORE_HIGH.INTAKE_ANGLE);
        frc::SmartDashboard::PutNumber("hp e", curGPInfo.HP_INTAKE.ELEVATOR_LENG);
        frc::SmartDashboard::PutNumber("hp w", curGPInfo.HP_INTAKE.INTAKE_ANGLE);
        frc::SmartDashboard::PutNumber("grnd e", curGPInfo.GROUND_INTAKE.ELEVATOR_LENG);
        frc::SmartDashboard::PutNumber("grnd w", curGPInfo.GROUND_INTAKE.INTAKE_ANGLE);
    }
}

void ElevatorIntake::Init() {
    m_elevator.Init();
}

ElevatorIntake::MovingState ElevatorIntake::GetState(){
    return m_movingState;
}

void ElevatorIntake::UpdateShuffleboard() {
    m_elevator.UpdateShuffleboard();
}

void ElevatorIntake::DeployElevatorIntake(double elevatorLength, double intakeAng){
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = elevatorLength;
    m_targIntakeAng = intakeAng;
    m_startElevatorPos = m_elevator.GetPos();
    m_startIntakeAng = m_intake.GetPos();
    // m_intake.SetHPIntake(false);
}

void ElevatorIntake::Stow(){
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = ElevatorConstants::STOWED_HEIGHT;
    m_targIntakeAng =  IntakeConstants::STOWED_POS;
    m_startElevatorPos = m_elevator.GetPos();
    m_startIntakeAng = m_intake.GetPos();
    m_targState = STOWED;
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
    // m_outtaking = frc::SmartDashboard::GetBoolean("outtake", false);
    // m_cone = frc::SmartDashboard::GetBoolean("cone", false);
    frc::SmartDashboard::PutNumber("elevator acc pos", m_elevator.getElevatorHeight());
    frc::SmartDashboard::PutNumber("intake acc angle", m_intake.GetPos());

    frc::SmartDashboard::PutNumber("moving state", m_movingState);
    frc::SmartDashboard::PutNumber("mechanism state", m_state);

    frc::SmartDashboard::PutString("el state", m_elevator.getStateString());

    frc::SmartDashboard::PutNumber("intake targ state", m_intake.GetTargetState());
    frc::SmartDashboard::PutNumber("intake state", m_intake.GetState());
    frc::SmartDashboard::PutString("elevator state", m_elevator.getStateString());
    frc::SmartDashboard::PutString("elevator target", m_elevator.getTargetString());
}

void ElevatorIntake::DebugScoring(){
    // frc::SmartDashboard::PutBoolean("outtake", m_outtaking);
    // frc::SmartDashboard::PutBoolean("cone", m_cone);
    frc::SmartDashboard::PutNumber("elevator acc pos", m_elevator.getElevatorHeight());
    frc::SmartDashboard::PutNumber("intake acc angle", m_intake.GetPos());
    frc::SmartDashboard::PutNumber("elevator targ pos", m_targElevatorPos);
    frc::SmartDashboard::PutNumber("intake targ angle", m_targIntakeAng);
    frc::SmartDashboard::PutBoolean("rollers", m_rollers);

    curGPInfo = {{frc::SmartDashboard::GetNumber("low e", curGPInfo.SCORE_LOW.ELEVATOR_LENG), 
    frc::SmartDashboard::GetNumber("low w", curGPInfo.SCORE_LOW.INTAKE_ANGLE)},{
    frc::SmartDashboard::GetNumber("mid e", curGPInfo.SCORE_MID.ELEVATOR_LENG),
    frc::SmartDashboard::GetNumber("mid w", curGPInfo.SCORE_MID.INTAKE_ANGLE)},{
    frc::SmartDashboard::GetNumber("high e", curGPInfo.SCORE_HIGH.ELEVATOR_LENG),
    frc::SmartDashboard::GetNumber("high w", curGPInfo.SCORE_HIGH.INTAKE_ANGLE)},{
    frc::SmartDashboard::GetNumber("grnd e", curGPInfo.GROUND_INTAKE.ELEVATOR_LENG),
    frc::SmartDashboard::GetNumber("grnd w", curGPInfo.GROUND_INTAKE.INTAKE_ANGLE)},{
    frc::SmartDashboard::GetNumber("hp e", curGPInfo.HP_INTAKE.ELEVATOR_LENG),
    frc::SmartDashboard::GetNumber("hp w", curGPInfo.HP_INTAKE.INTAKE_ANGLE)}};
}

void ElevatorIntake::Periodic(){
    m_intake.Periodic();
    m_elevator.Periodic();
}

void ElevatorIntake::ToggleRoller(bool outtaking){
    if (m_rollers){
        if (m_outtaking != outtaking){
            m_intake.StartRollers(outtaking, m_cone);
        } else {
            m_intake.StopRollers();
            m_rollers = false;
        }
    } else {
        m_intake.StartRollers(outtaking, m_cone); 
        m_rollers = true;
    }
    m_outtaking = outtaking;
}

void ElevatorIntake::UpdateLidarData(LidarReader::LidarData& lidarData){
    m_intake.UpdateLidarData(lidarData);
}

void ElevatorIntake::TeleopPeriodic(){
   if (dbg) {
    Debug();
    DebugScoring();
   }

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
                        if(m_targState == STOWED)
                            m_elevator.Stow();
                        else
                            m_elevator.ExtendToCustomPos(m_targElevatorPos);
                        m_movingState = ELEVATOR;
                    }
                    break;
                case ELEVATOR:
                    if (m_elevator.getState() == Elevator::HOLDING_POS){    
                        if (m_targState == STOWED){
                            m_intake.Stow();
                        } else {
                            m_intake.ChangeDeployPos(m_targIntakeAng);
                            m_intake.DeployNoRollers(m_targState == HP);
                            if (m_rollers)
                                m_intake.StartRollers(m_outtaking, m_cone);
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

void ElevatorIntake::SetCone(bool cone){
    m_cone = cone;
    frc::SmartDashboard::PutBoolean("cone", m_cone);
}

void ElevatorIntake::ScoreHigh(){
    m_targState = HIGH;
    DeployElevatorIntake(GetGPI(m_cone).SCORE_HIGH);
}

double ElevatorIntake::GetWristCompletion(){
    return fabs(m_targIntakeAng - m_intake.GetPos())/fabs(m_targIntakeAng-m_startIntakeAng);
}
double ElevatorIntake::GetElevatorCompletion(){
    return fabs(m_targElevatorPos - m_elevator.GetPos())/fabs(m_targElevatorPos-m_startIntakeAng);
}

void ElevatorIntake::ScoreMid(){
    m_targState = MID;
    DeployElevatorIntake(GetGPI(m_cone).SCORE_MID);
}
void ElevatorIntake::ScoreLow(){
    m_targState = LOW;
   DeployElevatorIntake(GetGPI(m_cone).SCORE_LOW);
}

void ElevatorIntake::IntakeFromGround(){
    m_targState = GROUND;
   DeployElevatorIntake(GetGPI(m_cone).GROUND_INTAKE);
}

void ElevatorIntake::IntakeFromHPS(){
    m_targState = HP;
   DeployElevatorIntake(GetGPI(m_cone).HP_INTAKE);
//    m_intake.SetHPIntake(true);
}

IntakeElevatorConstants::GamePieceInfo ElevatorIntake::GetGPI(bool cone){
    if (dbg2){
        if (cone) return coneinfo;
        return cubeinfo;
    } else 
        if (cone) return IntakeElevatorConstants::coneScoreInfo;
        return IntakeElevatorConstants::cubeScoreInfo;
}

void ElevatorIntake::DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo){
    DeployElevatorIntake(scoreInfo.ELEVATOR_LENG, scoreInfo.INTAKE_ANGLE);
}

/**
 * Manual periodic
 * 
 * @note call this instead of teleop periodic
 * 
 * @param elevator -1 to 1, percent of elevator max volts
 * @param intake -1 to 1, percent of intake max volts
*/
void ElevatorIntake::ManualPeriodic(double elevator, double intake) {
   if (dbg) Debug();
   if (dbg2) DebugScoring();
    m_elevator.setManualVolts(elevator);
    m_intake.ManualPeriodic(intake * IntakeConstants::WRIST_MAX_VOLTS);
    m_elevator.TeleopPeriodic();
}

/**
 * Determines if elevator & intake can move fast
*/
bool ElevatorIntake::CanMoveFast() const {
    return (m_movingState == DONE) &&
        (m_targState == STOWED || m_targState == HP || m_targState == GROUND);
}