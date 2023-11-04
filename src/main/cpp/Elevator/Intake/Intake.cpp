#include "Elevator/Intake/Intake.h"
#include <iostream>

//constructor j sets motor to brake mode
Intake::Intake(std::string name, bool enabled, bool shuffleboard):
    Mechanism(name, enabled, shuffleboard)
{
    m_wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

// needs to be called INSTEAD of teleop periodic
void Intake::setManualVolts(double wristVolts){
    m_state = MANUAL;
    m_manualVolts = wristVolts;
}

void Intake::CorePeriodic(){
    UpdatePose();
}

// teleop periodic runs on state machine
void Intake::CoreTeleopPeriodic(){
    double wristVolts = 0;
    double spikeCur;
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
            }
            break;
        case AT_TARGET:
            wristVolts = FFPIDCalculate();
            break;
        case MANUAL:
            wristVolts = m_manualVolts;
    }
    if (shuff_.isEnabled()){
        shuff_.PutNumber("wrist volts", wristVolts, {2,1,0,1});
    }
    m_wristMotor.SetVoltage(units::volt_t(std::clamp(-wristVolts, -IntakeConstants::WRIST_MAX_VOLTS, IntakeConstants::WRIST_MAX_VOLTS)));
}


//completely stows the intake at its maximum position
void Intake::Stow(){
    if (m_targState == STOWED) return;
    m_targState = STOWED;
    SetSetpoint(IntakeConstants::STOWED_POS);
    m_state = MOVING;
}

// half-stows the intake, moving it out of the way of the elevator
void Intake::HalfStow(){
    if (m_targState == HALFSTOWED) return;
    m_targState = HALFSTOWED;
    SetSetpoint(IntakeConstants::INTAKE_UPRIGHT_ANGLE);
    m_state = MOVING;
}

// deploys the intake to intake a cone or cube
void Intake::Deploy(){
    if (m_targState == DEPLOYED) return;
    m_targState = DEPLOYED;

    if (m_customDeployPos == -1)
        m_setPt = IntakeConstants::DEPLOYED_POS;
    else 
        m_setPt = m_customDeployPos;

    SetSetpoint(m_setPt);
    m_state = MOVING;
}

//changes the position the intake will deploy to when a deploy method iscalled
//so not an action method
void Intake::ChangeDeployPos(double newPos){
    m_customDeployPos = std::clamp(newPos, IntakeConstants::MIN_POS, IntakeConstants::MAX_POS);
}

//disables intake
void Intake::Kill(){
    m_state = STOPPED;
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

//the following functions are all private methods

//Updates the current position, velocity, and acceleration of the wrist
void Intake::UpdatePose(){
    double newPos = m_wristEncoder.GetAbsolutePosition() * 2 * M_PI + IntakeConstants::WRIST_ABS_ENCODER_OFFSET; // might need to negate or do some wrap around calculations
    double newVel = (newPos - m_curPos)/0.02;
    m_curAcc = (newVel - m_curVel)/0.02;
    m_curVel = newVel;
    m_curPos = newPos;
}

//updates the trapezoidal motion profile
void Intake::UpdateTargetPose(){
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

//calculates voltage output with feedforwardPID
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
    if (shuff_.isEnabled()){
        shuff_.PutNumber("posErr", posErr, {1,1,0,5}); 
        shuff_.PutNumber("velErr", velErr, {1,1,1,5}); 
        shuff_.PutNumber("ff out", ff, {1,1,2,5}); 
        shuff_.PutNumber("pid out", pid, {1,1,3,5});
    }
    return pid+ff;
}

//calculates the position at which the speed will begin decreasing
void Intake::CalcSpeedDecreasePos(){
    double MAX_VEL = IntakeConstants::WRIST_MAX_VEL, MAX_ACC = IntakeConstants::WRIST_MAX_ACC;
    if(fabs(m_setPt - m_curPos) < MAX_VEL*MAX_VEL/MAX_ACC){ // for triangle motion profile
        m_speedDecreasePos = (m_setPt+m_curPos)/2;
    } else if (m_setPt > m_curPos)
        m_speedDecreasePos = m_setPt - MAX_VEL*MAX_VEL/(MAX_ACC*2);
    else 
        m_speedDecreasePos = m_setPt + MAX_VEL*MAX_VEL/(MAX_ACC*2);
}

//changes the setpoint and sets everything up to start the profile
void Intake::SetSetpoint(double setpt){
      m_setPt = setpt;
      m_targetPos = m_curPos;
      m_targetVel = 0.0;
      m_targetAcc = IntakeConstants::WRIST_MAX_ACC;
      if (m_setPt < m_curPos) m_targetAcc *= -1;
      ResetPID();
      CalcSpeedDecreasePos();
}

//returns whether the wrist is at its setpoint
bool Intake::AtSetpoint(){
    if (fabs(m_curPos - m_setPt) <= IntakeConstants::WRIST_POS_TOLERANCE)
        return true;
    return false;
}

void Intake::ResetPID(){
    m_totalErr = 0;
}



void Intake::CoreShuffleboardInit(){
    //Current Pose
    shuff_.add("cur pos", &m_curPos, {1,1,0,0});
    shuff_.add("cur vel", &m_curVel, {1,1,1,0});
    shuff_.add("cur acc", &m_curAcc, {1,1,2,0});

    //Setpoint
    shuff_.add("wrist setpt", &m_setPt, {2,1,4,2}, true);
    shuff_.add("manual volts", &m_manualVolts, {2,1,5,2}, true);

    //Deploy Button
    shuff_.addButton("Deploy", [&](){
                                    ChangeDeployPos(m_setPt);
                                    Deploy();
                                    }, {2,2,4,0});
    shuff_.addButton("Manual", [&](){
                                    setManualVolts(m_manualVolts);
                                    }, {2,2,6,0});

    //Feedforward
    shuff_.add("g", &m_g, {1,1,0,3}, true); 
    shuff_.add("s", &m_s, {1,1,1,3}, true); 
    shuff_.add("v", &m_v, {1,1,2,3}, true); 
    shuff_.add("a", &m_a, {1,1,3,3}, true);

    shuff_.add("targ vel", &m_targetVel, {1,1,0,4});
    shuff_.add("targ pos", &m_targetPos, {1,1,1,4});
    shuff_.add("targ acc", &m_targetAcc, {1,1,2,4});
}

void Intake::CoreShuffleboardPeriodic(){
    
}

void Intake::CoreShuffleboardUpdate(){
    
}