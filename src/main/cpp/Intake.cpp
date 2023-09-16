#include "Intake/Intake.h"

/*
ASSUMES THAT INTAKE IS STOWED WHEN INTIALIZED
*/
Intake::Intake(){

}
void Intake::UpdatePose(){
    m_curPos = StepsToRad(m_wristMotor.GetSelectedSensorPosition());
    double newVel = StepsToRad(m_wristMotor.GetSelectedSensorVelocity());
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
            break;
        case DEPLOYED:
            m_wristMotor.SetVoltage(units::volt_t(std::clamp(FFPIDCalculate(), -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
            double rollerVolts = IntakeConstants::ROLLER_MAX_VOLTS;
            if (m_outtaking) rollerVolts *= -1; // double check this idk which direction intake is
            m_rollerMotor.SetVoltage(units::volt_t(rollerVolts));
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

void Intake::Stow(){
    m_setPt = IntakeConstants::STOWED_POS;
    CalcVelTurnPos();
    ResetPID();
    m_state = STOWING;
}

void Intake::DeployIntake(){
    m_setPt = IntakeConstants::DEPLOYED_POS;
    m_outtaking = false;
    CalcVelTurnPos();
    ResetPID();
    m_state = DEPLOYING;
}

void Intake::DeployOuttake(){
    DeployIntake();
    m_outtaking = true;
}

void Intake::Kill(){
    m_state = STOPPED;
}

// Converts steps to radians for falcons
double Intake::StepsToRad(double steps){
  return steps * (2.0 * M_PI / 2048.0); // falcon is 2048 ticks per revolution
}

void Intake::ResetPID(){
    m_totalErr = 0;
}

void Intake::CalcVelTurnPos(){
    double MAX_VEL = IntakeConstants::EXTEND_DEPLOY_MAX_VEL, MAX_ACC = IntakeConstants::EXTEND_DEPLOY_MAX_ACC;
    if (m_curPos > m_targetPos)
        m_velTurnPos = MAX_VEL*(MAX_VEL - 2.0)/(MAX_ACC*2) + m_setPt;
    else 
        m_velTurnPos = -MAX_VEL*(MAX_VEL - 2.0)/(MAX_ACC*2) + m_setPt;
}

void Intake::UpdateTargetPose(){
    if (dbg){
        frc::SmartDashboard::PutNumber("targ pos", m_targetPos);
        frc::SmartDashboard::PutNumber("targ vel", m_targetVel);
        // frc::SmartDashboard::PutNumber("targ acc", m_targetAcc);
    }
    double newP = m_targetPos, newV = m_targetVel, newA = m_targetAcc;
    newP += m_targetVel;

    if (m_velTurnPos > 0){ // if trapezoid is pos
        if (m_targetPos > m_velTurnPos) //target is current, weird ik
            newV = std::max(0.0, newV - IntakeConstants::EXTEND_DEPLOY_MAX_ACC * 0.02);
        else 
            newV = std::min(IntakeConstants::EXTEND_DEPLOY_MAX_VEL, newV + IntakeConstants::EXTEND_DEPLOY_MAX_ACC * 0.02);
    } else {
        if (m_targetPos > m_velTurnPos) // if "before" the turn pt
            newV = std::max(-IntakeConstants::EXTEND_DEPLOY_MAX_VEL, newV - IntakeConstants::EXTEND_DEPLOY_MAX_ACC * 0.02);
        else 
            newV = std::min(0.0, newV + IntakeConstants::EXTEND_DEPLOY_MAX_ACC * 0.02);
    }

    if (newV-m_curVel == 0) newA = 0;
    else if (newV > m_curVel) newA = IntakeConstants::EXTEND_DEPLOY_MAX_ACC;
    else newA = -IntakeConstants::EXTEND_DEPLOY_MAX_ACC;

    m_targetPos = newP;
    m_targetVel = newV;
    m_targetAcc = newA;
}

