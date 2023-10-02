#include "Intake/Intake.h"

Intake::Intake(){}

void Intake::debugTargPose(){
    m_setPt = frc::SmartDashboard::GetNumber("Setpoint", m_setPt);
    frc::SmartDashboard::PutNumber("targ vel", m_targetVel);
    frc::SmartDashboard::PutNumber("targ pos", m_targetPos);
    frc::SmartDashboard::PutNumber("targ acc", m_targetAcc);
    std::string state;
    state = frc::SmartDashboard::GetString("State", "K");
    if (state == "DI")
        DeployIntake();
    else if (state == "DO") 
        DeployOuttake(); 
    else if (state == "S") 
        Stow();
    else if (state == "K") 
        Kill();
    
}

void Intake::debugCurPose(){
    frc::SmartDashboard::PutNumber("cur vel", m_curVel);
    frc::SmartDashboard::PutNumber("cur pos", m_curPos);
    frc::SmartDashboard::PutNumber("cur acc", m_curAcc);
}

void Intake::debugPutVoltage(){
    double voltReq;
    double WRIST_MAX_POS = 0.0,  WRIST_MIN_POS = 0.0;
    frc::SmartDashboard::GetNumber("voltage", voltReq);
    if(m_curPos > WRIST_MAX_POS){
        voltReq = 0;
    } else if(m_curPos < WRIST_MIN_POS){
        voltReq = 0;
    }
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(voltReq, -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
}

void Intake::UpdatePose(){
    double newPos = m_wristEncoder.GetAbsolutePosition() * 2 * M_PI + IntakeConstants::WRIST_ABS_ENCODER_OFFSET; // might need to negate or do some wrap around calculations
    double newVel = (newPos - m_curPos)/0.02;
    m_curAcc = (newVel - m_curVel)/0.02;
    m_curVel = newVel;
    m_curPos = newPos;
}

void Intake::TeleopPeriodic(){
    debugCurPose();
    // debugTargPose();
    // debugPutVoltage();

    UpdatePose();
    double wristVolts = 0, rollerVolts = 0;
    switch (m_state){
        case STOWED:
            wristVolts = m_stowedPIDcontroller.Calculate(IntakeConstants::STOWED_POS - m_curPos);
            break;
        case DEPLOYING:
        case STOWING:
            UpdateTargetPose(); // bc still using motion profile 
            wristVolts = FFPIDCalculate();
            if (AtSetpoint()){
                if (m_state == STOWING) m_state = STOWED;
                if (m_state == DEPLOYING) m_state = DEPLOYED;
                ResetPID();
            } else if (m_state == DEPLOYING) 
                rollerVolts = m_rollerVolts;
            break;
        case DEPLOYED:
            wristVolts = FFPIDCalculate();
            rollerVolts = m_rollerVolts;
            break;
    }

    frc::SmartDashboard::PutNumber("wrist volts", wristVolts);
    frc::SmartDashboard::PutNumber("roller volts", rollerVolts);
    // m_wristMotor.SetVoltage(units::volt_t(std::clamp(wristVolts, -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
    // m_rollerMotor.SetVoltage(units::volt_t(std::clamp(rollerVolts, -IntakeConstants::ROLLER_MAX_VOLTS,IntakeConstants::ROLLER_MAX_VOLTS)));
}

double Intake::FFPIDCalculate(){
    double posErr = m_targetPos - m_curPos, 
    velErr = m_targetVel - m_curVel;
    m_totalErr += posErr * 0.02;
    double pid = m_kp*posErr + m_kd*velErr + m_ki*m_totalErr;
    double ff = m_g* cos(m_targetPos) + m_s*((m_targetVel > 0) ? 1.0 : -1.0 ) + m_v*m_targetVel + m_a*m_targetAcc;
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

void Intake::DeployToCustomPos(double newPos){
    SetSetpoint(newPos);
    m_state = DEPLOYING;
}

void Intake::ChangeRollerVoltage(double newVoltage, bool outtake){
    m_rollerVolts = fabs(newVoltage);
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
      CalcSpeedDecreasePos();
}

bool Intake::AtSetpoint(){
    if (fabs(m_curPos - m_setPt) <= IntakeConstants::WRIST_POS_TOLERANCE)
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

void Intake::CalcSpeedDecreasePos(){
    double MAX_VEL = IntakeConstants::WRIST_MAX_VEL, MAX_ACC = IntakeConstants::WRIST_MAX_ACC;
    if(fabs(m_setPt - m_curPos) < MAX_VEL*MAX_VEL/MAX_ACC){ // for triangle motion profile
        m_speedDecreasePos = (m_setPt+m_curPos)/2;
    } else if (m_setPt > m_curPos)
        m_speedDecreasePos = m_setPt - MAX_VEL*MAX_VEL/(MAX_ACC*2);
    else 
        m_speedDecreasePos = m_setPt + MAX_VEL*MAX_VEL/(MAX_ACC*2);
}

void Intake::UpdateTargetPose(){
    if (dbg){
        frc::SmartDashboard::PutNumber("targ pos", m_targetPos);
        frc::SmartDashboard::PutNumber("targ vel", m_targetVel);
        // frc::SmartDashboard::PutNumber("targ acc", m_targetAcc);
    }
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    
    newP += m_targetVel * 0.02;

    if (m_speedDecreasePos < m_setPt){ // if trapezoid is pos
        if (newP > m_speedDecreasePos) // if after turn pt
            newV = std::max(0.0, m_targetVel - IntakeConstants::WRIST_MAX_ACC * 0.02);
        else 
            newV = std::min(IntakeConstants::WRIST_MAX_VEL, m_targetVel + IntakeConstants::WRIST_MAX_ACC * 0.02);
    } else {
        if (newP > m_speedDecreasePos) // if before the turn pt
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

