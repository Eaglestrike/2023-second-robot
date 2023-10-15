#include "Elevator/Intake/Intake.h"
#include <iostream>

//constructor j sets motor to brake mode
Intake::Intake(){
    m_wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    if (dbg){
        frc::SmartDashboard::PutNumber("Setpoint", 0);
        frc::SmartDashboard::PutBoolean("Deploy", false);
        frc::SmartDashboard::PutBoolean("Cone", false);
        frc::SmartDashboard::PutBoolean("Outtake", false);
        
        
        //m_wristMotor.SetInverted(true);
        frc::SmartDashboard::PutNumber("g", m_g); 
        frc::SmartDashboard::PutNumber("s", m_s); 
        frc::SmartDashboard::PutNumber("v", m_v); 
        frc::SmartDashboard::PutNumber("a", m_a); 
    }
    frc::SmartDashboard::PutNumber("voltage", 0.0); 
}

// to debug the trapezoidal motion profile
void Intake::debugTargPose(){ 
    ChangeDeployPos(frc::SmartDashboard::GetNumber("Setpoint", m_setPt));
    frc::SmartDashboard::PutNumber("targ vel", m_targetVel);
    frc::SmartDashboard::PutNumber("targ pos", m_targetPos);
    frc::SmartDashboard::PutNumber("targ acc", m_targetAcc);
    bool deploy, cone, outtake;
    deploy = frc::SmartDashboard::GetBoolean("Deploy", false);
    cone = frc::SmartDashboard::GetBoolean("Cone", false);
    outtake = frc::SmartDashboard::GetBoolean("Outtake", false);
    if (deploy){
        // ChangeDeployPos()
        if (outtake)
            DeployOuttake(cone);
        else 
            DeployIntake(cone);
        frc::SmartDashboard::PutBoolean("Deploy", false);
    }
}

// to debug the current pose calculations
void Intake::debugCurPose(){
    frc::SmartDashboard::PutNumber("cur vel", m_curVel);
    frc::SmartDashboard::PutNumber("cur pos", m_curPos);
    frc::SmartDashboard::PutNumber("cur acc", m_curAcc);
}

void Intake::DeployNoRollers(){
    if (m_targState == DEPLOYED) return;
    if (m_customDeployPos == -1)
        m_setPt = IntakeConstants::DEPLOYED_POS;
    else 
        m_setPt = m_customDeployPos;

    SetSetpoint(m_setPt);
    m_rollerVolts = 0;
    m_state = MOVING;
}

// for tuning, can test constant voltage on wrist or rollers 
// but need to pick which wrist or rollers in the code, since it cant be changed from shuffleboard
void Intake::debugPutVoltage(){
    double voltReq;
    voltReq = frc::SmartDashboard::GetNumber("voltage", voltReq);
    voltReq = std::clamp(voltReq, -IntakeConstants::ROLLER_MAX_VOLTS, IntakeConstants::ROLLER_MAX_VOLTS);
    // if(m_curPos > IntakeConstants::MAX_POS){
    //     voltReq = 0;
    // } else if(m_curPos < IntakeConstants::MIN_POS){
    //     voltReq = 0;
    // }
    std::cout << voltReq << std::endl ;

    frc::SmartDashboard::PutNumber("roller current", m_rollerMotor.GetOutputCurrent());

    m_rollerMotor.SetVoltage(units::volt_t(voltReq));
    //m_wristMotor.SetVoltage(units::volt_t(-voltReq));
}

//Updates the current position, velocity, and acceleration of the wrist
void Intake::UpdatePose(){
    double newPos = m_wristEncoder.GetAbsolutePosition() * 2 * M_PI + IntakeConstants::WRIST_ABS_ENCODER_OFFSET; // might need to negate or do some wrap around calculations
    double newVel = (newPos - m_curPos)/0.02;
    m_curAcc = (newVel - m_curVel)/0.02;
    m_curVel = newVel;
    m_curPos = newPos;
}

// needs to be called INSTEAD of teleop periodic
void Intake::ManualPeriodic(double wristVolts){
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
}

// teleop periodic runs on state machine
void Intake::TeleopPeriodic(){
    if (dbg){
        //debugCurPose();
        //debugTargPose();    
    }

    debugPutVoltage();

    UpdatePose();
    double wristVolts = 0, rollerVolts = 0;
    switch (m_state){
        case MOVING:
            UpdateTargetPose(); // bc still using motion profile 
            wristVolts = FFPIDCalculate();
            if (AtSetpoint()){
                m_state = AT_TARGET;
                ResetPID();
                m_targetPos = m_setPt;
                m_targetVel = 0.0;
                m_targetAcc = 0.0;
            } else if (m_targState == DEPLOYED) 
                rollerVolts = m_rollerVolts;
            break;
        case AT_TARGET:
            wristVolts = FFPIDCalculate();
            if (m_targState != STOWED){
                rollerVolts = m_rollerVolts;
                if(m_rollerMotor.GetOutputCurrent() > IntakeConstants::NORMAL_CURRENT)
                    if (m_cone)
                        rollerVolts = IntakeConstants::KEEP_CONE_VOLTS;
                    else 
                        rollerVolts = IntakeConstants::KEEP_CUBE_VOLTS;
            }
            break;
    }
    if (dbg){
        frc::SmartDashboard::PutNumber("wrist volts", wristVolts);
        frc::SmartDashboard::PutNumber("roller volts", rollerVolts);
    } 
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
    //m_rollerMotor.SetVoltage(units::volt_t(std::clamp(rollerVolts, -IntakeConstants::ROLLER_MAX_VOLTS,IntakeConstants::ROLLER_MAX_VOLTS)));
}

double Intake::FFPIDCalculate(){
    double posErr = m_targetPos - m_curPos, 
    velErr = m_targetVel - m_curVel;
    m_totalErr += posErr * 0.02;
    if (fabs(posErr) <= IntakeConstants::WRIST_POS_TOLERANCE) posErr =0;
    double pid = m_kp*posErr + m_kd*velErr + m_ki*m_totalErr;
    double s = m_s;
    if (m_targetVel < 0) s = -m_s;
    else if (m_targetVel == 0) s = 0;
    double ff = m_g* cos(m_targetPos) + s + m_v*m_targetVel + m_a*m_targetAcc;
    if (dbg){
        frc::SmartDashboard::PutNumber("posErr", posErr); 
        frc::SmartDashboard::PutNumber("velErr", velErr); 
        frc::SmartDashboard::PutNumber("ff out", ff); 
        frc::SmartDashboard::PutNumber("pid out", pid); 
        //see how each term is contributing to ff
        m_g = frc::SmartDashboard::GetNumber("g", m_g); 
        m_s = frc::SmartDashboard::GetNumber("s", m_s); 
        m_v = frc::SmartDashboard::GetNumber("v", m_v); 
        m_a = frc::SmartDashboard::GetNumber("a", m_a); 
    }
    return pid+ff;
}

void Intake::ChangeDeployPos(double newPos){
    m_customDeployPos = std::clamp(newPos, IntakeConstants::MIN_POS, IntakeConstants::MAX_POS);
}

void Intake::ChangeRollerVoltage(double newVoltage){
    m_customRollerVolts = fabs(newVoltage);
}

void Intake::Stow(){
    if (m_targState == STOWED) return;
    m_targState = STOWED;
    SetSetpoint(IntakeConstants::STOWED_POS);
    m_rollerVolts = 0;
    m_state = MOVING;
}

void Intake::HalfStow(){
    if (m_targState == HALFSTOWED) return;
    m_targState = HALFSTOWED;
    SetSetpoint(IntakeConstants::INTAKE_UPRIGHT_ANGLE);
    m_rollerVolts = 0;
    m_state = MOVING;
}

void Intake::DeployIntake(bool cone){
    if (m_targState == DEPLOYED) return;
    m_targState = DEPLOYED;
    if (m_customDeployPos == -1)
        m_setPt = IntakeConstants::DEPLOYED_POS;
    else 
        m_setPt = m_customDeployPos;

    SetSetpoint(m_setPt);
    if(m_customRollerVolts == -1)
        m_rollerVolts = IntakeConstants::ROLLER_MAX_VOLTS;
    else 
        m_rollerVolts = m_customRollerVolts;

    if (cone)
        m_rollerVolts *= -1;
    
    m_cone = cone;
    m_state = MOVING;
}

void Intake::DeployOuttake(bool cone){
    DeployIntake(!cone);
    m_cone = cone;
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
        frc::SmartDashboard::PutNumber("targ acc", m_targetAcc);
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

Intake::MechState Intake::GetState(){
    return m_state;
}

Intake::TargetState Intake::GetTargetState(){
    return m_targState;
}

double Intake::GetPos(){
    return m_curPos;
}
