#include "Elevator/ElevatorIntake.h"

#include <iostream>

ElevatorIntake::ElevatorIntake(std::string name, bool enabled, bool shuffleboard):
    Mechanism(name, enabled, shuffleboard),
    m_intake("Intake", enabled, shuffleboard)
{
}

void ElevatorIntake::CoreInit() {
    m_intake.Init();
    m_elevator.Init();
}

void ElevatorIntake::CorePeriodic(){
    m_intake.Periodic();
    m_elevator.Periodic();
}

void ElevatorIntake::DeployElevatorIntake(double elevatorLength, double intakeAng){
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = elevatorLength;
    m_targIntakeAng = intakeAng;
    // m_intake.SetHPIntake(false);
}

void ElevatorIntake::Stow(){
    if (m_targState == STOWED) return;
    m_state = MOVING;
    m_movingState = HALFSTOWING;
    m_targElevatorPos = ElevatorConstants::STOWED_HEIGHT;
    m_targIntakeAng =  IntakeConstants::STOWED_POS;
    m_targState = STOWED;
}

void ElevatorIntake::CoreTeleopPeriodic(){
    frc::SmartDashboard::PutNumber("elevator acc pos", m_elevator.getElevatorHeight());
    frc::SmartDashboard::PutNumber("intake acc angle", m_intake.GetPos());
    frc::SmartDashboard::PutNumber("elevator targ pos", m_targElevatorPos);
    frc::SmartDashboard::PutNumber("intake targ angle", m_targIntakeAng);    
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
                        else{
                            // std::cout << "extending elevator " << std::endl;
                            m_elevator.ExtendToCustomPos(m_targElevatorPos);
                        }
                           
                        m_movingState = ELEVATOR;
                    }
                    break;
                case ELEVATOR:
                    if (m_elevator.getState() == Elevator::HOLDING_POS){    
                        if (m_targState == STOWED){
                            m_intake.Stow();
                        } else {
                            m_intake.ChangeDeployPos(m_targIntakeAng);
                            m_intake.Deploy();
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
        case MANUAL:
            m_elevator.setManualVolts(m_manualElevator);
    }
}

bool ElevatorIntake::IsDone() const {
    return m_movingState == DONE;
}

void ElevatorIntake::Kill(){
    m_state = STOPPED;
}

void ElevatorIntake::SetCone(bool cone){
    m_cone = cone;
    frc::SmartDashboard::PutBoolean("cone", m_cone);
}

void ElevatorIntake::ScoreHigh(){
    if (m_targState == HIGH) return;
    // std::cout << "scoring high" << std::endl;
    m_targState = HIGH;
    DeployElevatorIntake(GetGPI(m_cone).SCORE_HIGH);
}

void ElevatorIntake::ScoreMid(){
    if (m_targState == MID) return;
    m_targState = MID;
    DeployElevatorIntake(GetGPI(m_cone).SCORE_MID);
}
void ElevatorIntake::ScoreLow(){
    if (m_targState == LOW) return;
    m_targState = LOW;
   DeployElevatorIntake(GetGPI(m_cone).SCORE_LOW);
}

void ElevatorIntake::IntakeFromGround(){
    if (m_targState == GROUND) return;
    m_targState = GROUND;
    m_cone = false;
   DeployElevatorIntake(GetGPI(m_cone).GROUND_INTAKE);
}

/**
 * Ground Flange Cone intake
*/
void ElevatorIntake::IntakeFlange(){
    m_targState = GROUND;
   DeployElevatorIntake(GetGPI(true).FLANGE_INTAKE);
}

void ElevatorIntake::IntakeFromHPS(){
    if(m_targState == HP) return;
    m_targState = HP;
    m_cone = true;
    DeployElevatorIntake(GetGPI(m_cone).HP_INTAKE);
//    m_intake.SetHPIntake(true);
}

IntakeElevatorConstants::GamePieceInfo ElevatorIntake::GetGPI(bool cone){
    if (cone) return coneinfo;
    return cubeinfo;
}

void ElevatorIntake::DeployElevatorIntake(IntakeElevatorConstants::ElevatorIntakePosInfo scoreInfo){
    // std::cout << "el length: " << scoreInfo.ELEVATOR_LENG << " + ang: " << scoreInfo.INTAKE_ANGLE << std::endl;
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
void ElevatorIntake::SetManualVolts(double elevator, double intake) {
    m_state = MANUAL;
    m_elevator.setManualVolts(elevator);
    m_intake.setManualVolts(intake * IntakeConstants::WRIST_MAX_VOLTS);
}

/**
 * Determines if elevator & intake can move fast
*/
bool ElevatorIntake::CanMoveFast() const {
    return (m_movingState == DONE) &&
        (m_targState == STOWED || m_targState == HP || m_targState == GROUND);
}

void ElevatorIntake::CoreShuffleboardInit(){
    //Target
    shuff_.add("Target Elevator Pos ", &m_targElevatorPos, {2,1,0,0}, true);
    shuff_.add("Target Intake Pos ", &m_targIntakeAng, {2,1,2,0}, true);

    //Buttons
    shuff_.addButton("Deploy", [&](){DeployElevatorIntake(m_targElevatorPos, m_targIntakeAng);}, {1,1,4,0});
    shuff_.addButton("Stow", [&](){Stow();}, {1,1,5,0});

    //Status
    shuff_.add("Cone", &m_cone, {1,1,1,1}, true);

    //Scoring pos
    shuff_.PutNumber("low e", curGPInfo.SCORE_LOW.ELEVATOR_LENG, {1,1,0,5});
    shuff_.PutNumber("low w", curGPInfo.SCORE_LOW.INTAKE_ANGLE, {1,1,1,5});
    shuff_.PutNumber("mid e", curGPInfo.SCORE_MID.ELEVATOR_LENG, {1,1,2,5});
    shuff_.PutNumber("mid w", curGPInfo.SCORE_MID.INTAKE_ANGLE, {1,1,3,5});
    shuff_.PutNumber("high e", curGPInfo.SCORE_HIGH.ELEVATOR_LENG, {1,1,4,5});
    shuff_.PutNumber("high w", curGPInfo.SCORE_HIGH.INTAKE_ANGLE, {1,1,5,5});
    shuff_.PutNumber("hp e", curGPInfo.HP_INTAKE.ELEVATOR_LENG, {1,1,6,5});
    shuff_.PutNumber("hp w", curGPInfo.HP_INTAKE.INTAKE_ANGLE, {1,1,7,5});
    shuff_.PutNumber("grnd e", curGPInfo.GROUND_INTAKE.ELEVATOR_LENG, {1,1,8,5});
    shuff_.PutNumber("grnd w", curGPInfo.GROUND_INTAKE.INTAKE_ANGLE, {1,1,9,5});
}

void ElevatorIntake::CoreShuffleboardPeriodic(){
    //Positions
    shuff_.PutNumber("elevator pos", m_elevator.getElevatorHeight(), {1,1,0,3});
    shuff_.PutNumber("intake angle", m_intake.GetPos(), {1,1,1,3});

    //States
    shuff_.PutNumber("moving state", m_movingState, {2,1,4,3});
    shuff_.PutNumber("mechanism state", m_state, {2,1,6,3});

    shuff_.PutString("elevator state", m_elevator.getStateString(), {2,1,0,4});
    shuff_.PutString("elevator targ state", m_elevator.getTargetString(), {2,1,2,4});
    shuff_.PutNumber("intake targ state", m_intake.GetTargetState(), {2,1,4,4});
    shuff_.PutNumber("intake state", m_intake.GetState(), {2,1,6,4});

    //Scoring Positions
    curGPInfo = {{shuff_.GetNumber("low e", curGPInfo.SCORE_LOW.ELEVATOR_LENG), 
                shuff_.GetNumber("low w", curGPInfo.SCORE_LOW.INTAKE_ANGLE)},{
                shuff_.GetNumber("mid e", curGPInfo.SCORE_MID.ELEVATOR_LENG),
                shuff_.GetNumber("mid w", curGPInfo.SCORE_MID.INTAKE_ANGLE)},{
                shuff_.GetNumber("high e", curGPInfo.SCORE_HIGH.ELEVATOR_LENG),
                shuff_.GetNumber("high w", curGPInfo.SCORE_HIGH.INTAKE_ANGLE)},{
                shuff_.GetNumber("grnd e", curGPInfo.GROUND_INTAKE.ELEVATOR_LENG),
                shuff_.GetNumber("grnd w", curGPInfo.GROUND_INTAKE.INTAKE_ANGLE)},{
                shuff_.GetNumber("hp e", curGPInfo.HP_INTAKE.ELEVATOR_LENG),
                shuff_.GetNumber("hp w", curGPInfo.HP_INTAKE.INTAKE_ANGLE)}};
}

void ElevatorIntake::CoreShuffleboardUpdate() {
    m_elevator.UpdateShuffleboard();
}