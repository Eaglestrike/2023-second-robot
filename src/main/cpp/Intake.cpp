#include "Intake/Intake.h"

/*
todo: make roller speed variable,
make angle variable
offset so that 0 is parallel w/ ground extended
*/
Intake::Intake(){}

//TODO:: Convert encoder steps to rads;
void Intake::UpdatePose(){
    m_curPos = m_wristEncoder.GetAbsolutePosition() * 2 * M_PI + IntakeConstants::WRIST_ABS_ENCODER_OFFSET; // might need to negate or do some wrap around calculations
    double stepsPerSec = m_wristMotor.GetSelectedSensorVelocity() * 10; // fn returns steps per 100 ms so multiply by 1/10 for per sec
                                                                        //x steps/100ms * 1000 ms/1sec = 10x steps /sec
    double newVel = StepsToRad(stepsPerSec);
    m_curAcc = (newVel - m_curVel)/0.02;
    m_curVel = newVel;
}

void Intake::TeleopPeriodic(){
    UpdatePose();
    switch (m_state){
        case STOPPED:
        case STOWED:
            m_wristMotor.SetVoltage(units::volt_t(0));
            m_rollerMotor.SetVoltage(units::volt_t(0));
            break;
        case STOWING:
        case DEPLOYING:
            UpdateTargetPose(); // bc still using motion profile 
            m_wristMotor.SetVoltage(units::volt_t(std::clamp(FFPIDCalculate(), -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
            if (AtSetpoint()){
                if (m_state == STOWING) m_state = STOWED;
                if (m_state == DEPLOYING) m_state = DEPLOYED;
            }
            break;
        case DEPLOYED:
            m_wristMotor.SetVoltage(units::volt_t(std::clamp(FFPIDCalculate(), -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
            m_rollerMotor.SetVoltage(units::volt_t(std::clamp(m_rollerVolts, -IntakeConstants::ROLLER_MAX_VOLTS,IntakeConstants::ROLLER_MAX_VOLTS)));
            break;
    }
}

double Intake::FFPIDCalculate(){
    double posErr = m_targetPos - m_curPos, 
    velErr = m_targetVel - m_curVel;
    m_totalErr += posErr;
    double pid = m_kp*posErr + m_kd*velErr + m_ki*m_totalErr;
    double ff = m_g + m_s*m_targetPos + m_v*m_targetVel + m_a*m_targetAcc;
    if (dbg){
        frc::SmartDashboard::PutNumber("posErr", posErr); 
        frc::SmartDashboard::PutNumber("velErr", velErr); 
        frc::SmartDashboard::PutNumber("ff out", ff); 
        frc::SmartDashboard::PutNumber("pid out", pid); 
        //see how each term is contributing to ff
        frc::SmartDashboard::PutNumber("g", m_g); 
        frc::SmartDashboard::PutNumber("s", m_s * m_targetPos); 
        frc::SmartDashboard::PutNumber("v", m_v * m_targetVel); 
        frc::SmartDashboard::PutNumber("a", m_a * m_targetAcc); 
    }
    return pid+ff;
}

void Intake::ChangeWristPos(double newPos){
    SetSetpoint(newPos);
    m_state = DEPLOYING;
}

void Intake::ChangeRollerVoltage(double newVoltage, bool outtake){
    m_rollerVolts = abs(newVoltage);
    if (outtake) m_rollerVolts *= -1;
}

void Intake::Stow(){
    SetSetpoint(IntakeConstants::STOWED_POS);
    m_rollerVolts = 0;
    m_state = STOWING;
}

void Intake::DeployIntake(){
    SetSetpoint(IntakeConstants::DEPLOYED_POS);
    m_rollerVolts = IntakeConstants::ROLLER_MAX_VOLTS;
    m_state = DEPLOYING;
}

void Intake::DeployOuttake(){
    SetSetpoint(IntakeConstants::DEPLOYED_POS);
    m_rollerVolts = -IntakeConstants::ROLLER_MAX_VOLTS;
    m_state = DEPLOYING;
}

void Intake::Kill(){
    m_state = STOPPED;
}

void Intake::SetSetpoint(double setpt){
      m_setPt = setpt;
      m_targetPos = m_curPos;
      m_targetVel = 0.0;
      m_targetAcc = IntakeConstants::WRIST_MAX_ACC;
      if (m_setPt < m_curPos) m_targetAcc *= -1;
      ResetPID();
      CalcVelTurnPos();
}

bool Intake::AtSetpoint(){
    if (abs(m_curPos - m_setPt) <= IntakeConstants::WRIST_POS_TOLERANCE)
        return true;
    return false;
}

// Converts steps to radians for the hex bore encoder
double Intake::StepsToRad(double steps){
  return steps * (2.0 * M_PI / 2048.0); // falcon is 2048 ticks per revolution
}

void Intake::ResetPID(){
    m_totalErr = 0;
}

void Intake::CalcVelTurnPos(){
    double MAX_VEL = IntakeConstants::WRIST_MAX_VEL, MAX_ACC = IntakeConstants::WRIST_MAX_ACC;
    if(abs(m_setPt - m_curPos) < MAX_VEL*MAX_VEL/MAX_ACC){ // for triangle motion profile
        m_velTurnPos = (m_setPt+m_curPos)/2;
    } else if (m_setPt > m_curPos)
        m_velTurnPos = m_setPt - MAX_VEL*MAX_VEL/(MAX_ACC*2);
    else 
        m_velTurnPos = m_setPt + MAX_VEL*MAX_VEL/(MAX_ACC*2);
}

void Intake::UpdateTargetPose(){
    if (dbg){
        frc::SmartDashboard::PutNumber("targ pos", m_targetPos);
        frc::SmartDashboard::PutNumber("targ vel", m_targetVel);
        // frc::SmartDashboard::PutNumber("targ acc", m_targetAcc);
    }
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    
    newP += m_targetVel;

    if (m_velTurnPos < m_setPt){ // if trapezoid is pos
        if (m_targetPos > m_velTurnPos) // if after turn pt
            newV = std::max(0.0, m_targetVel - IntakeConstants::WRIST_MAX_ACC * 0.02);
        else 
            newV = std::min(IntakeConstants::WRIST_MAX_VEL, m_targetVel + IntakeConstants::WRIST_MAX_ACC * 0.02);
    } else {
        if (m_targetPos > m_velTurnPos) // if before the turn pt
            newV = std::max(-IntakeConstants::WRIST_MAX_VEL, m_targetVel - IntakeConstants::WRIST_MAX_ACC * 0.02);
        else 
            newV = std::min(0.0, m_targetVel + IntakeConstants::WRIST_MAX_ACC * 0.02);
    }

    if (newV-m_targetVel == 0) newA = 0;
    else if (newV > m_targetVel) newA = IntakeConstants::WRIST_MAX_ACC;
    else newA = -IntakeConstants::WRIST_MAX_ACC;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;
}

