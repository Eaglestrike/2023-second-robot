#include "Climb/Climb.h"

Climb::Climb() {
  m_regPID.SetTolerance(ClimbConstants::EXTND_STOW_POS_ERR_TOLERANCE,
                            ClimbConstants::EXTND_STOW_VEL_ERR_TOLERANCE);
  if (ClimbConstants::FEEDFORWARD){
    m_FFPID.SetFeedForwardConsts(ClimbConstants::FF_S,
                            ClimbConstants::FF_G, ClimbConstants::FF_V, ClimbConstants::FF_A);
    m_FFPID.SetPIDConsts(ClimbConstants::CLIMB_P, ClimbConstants::CLIMB_D);
  } else {
    m_climbPID.SetTolerance(ClimbConstants::CLIMB_POS_ERR_TOLERANCE, ClimbConstants::CLIMB_VEL_ERR_TOLERANCE);
  }
}

// assumes climb is stowed, and zeros encoders accordingly
void Climb::RobotInit(){
  frc::SmartDashboard::PutNumber("volts", vlts);
  ZeroEncoder();
  ResetPIDs();
}

// calculates motor position in rad
void Climb::UpdatePos(){
 double steps = m_motor.GetSelectedSensorPosition(); 
 m_currentPos = StepsToRad(steps);
 if (dbg){
  frc::SmartDashboard::PutNumber("cur pos", m_currentPos);
  }
}

// calculates motor velocity in rad/sec
// and acceleration in rad/sec^2 using the old velocity value 
void Climb::UpdateVelAcc(){
 double stepsPerSec = m_motor.GetSelectedSensorVelocity() * 10; // fn returns steps per 100 ms so multiply by 10 for per sec
 double newVel = StepsToRad(stepsPerSec);
 m_currentAcc = (newVel - m_currentVel)/0.02;  // periodic is called every 20 ms
 m_currentVel = newVel;
 if (dbg){
  frc::SmartDashboard::PutNumber("cur vel", m_currentVel);
  frc::SmartDashboard::PutNumber("cur acc", m_currentVel);
  }
}

// Converts steps to radians for falcons
double Climb::StepsToRad(double steps){
  return steps * (2.0 * M_PI / 2048.0); // falcon is 2048 ticks per revolution
}

void Climb::CollectTuningData(){
  vlts = frc::SmartDashboard::GetNumber("volts", vlts);
  // vlts = frc::SmartDashboard::GetNumber("volts", vlts);

  UpdateVelAcc();
  // double curvel = m_motor.GetSelectedSensorVelocity();
  // if (curvel != 0.0){
  //   vel.push_back(curvel);
  //   volts.push_back(vlts);
  // }
  m_motor.SetVoltage((units::volt_t)vlts);
  // frc::SmartDashboard::PutNumberArray();
  return;
}

// This function runs during teleop periodic and does the following:
// Updates the position variable
// then based on the state and consequently the objective position at the moment
// itll use a controller (either PID or FF) to set the voltage
// in places where it uses PID it constantly checks wether its done with its task and changes the state accordingly
void Climb::TeleopPeriodic() {
  UpdatePos();
  UpdateVelAcc();
  if(dbg)
    frc::SmartDashboard::PutString("climb state:", StateToString());
  double controllerOut, volts, maxVolts;
  switch (m_state) {
    case STOWED:
    case EXTENDED:
      m_motor.SetVoltage((units::volt_t)0);
      break;
    case STOWING:
      controllerOut = m_regPID.Calculate(m_currentPos, ClimbConstants::STOWED_POS);
      maxVolts = ClimbConstants::EXTND_STOW_MAX_VOLTAGE;
      if (m_regPID.AtSetpoint())
        m_state = STOWED;
      break;
    case EXTENDING:
      controllerOut = m_regPID.Calculate(m_currentPos, ClimbConstants::EXTENDED_POS);
      maxVolts = ClimbConstants::EXTND_STOW_MAX_VOLTAGE;
      if (m_regPID.AtSetpoint())
        m_state = EXTENDED;
      break;
    case LIFTING:
      controllerOut;
      if (ClimbConstants::FEEDFORWARD){
        controllerOut = m_FFPID.Calculate(m_currentPos, m_currentVel); // returns volts
      } else {
        controllerOut = m_climbPID.Calculate(m_currentPos, ClimbConstants::LIFTED_POS);
      } 
      maxVolts = ClimbConstants::CLIMB_MAX_VOLTAGE;
       m_motor.SetVoltage((units::volt_t)volts);
      break;
  } 
  if (dbg)
  frc::SmartDashboard::PutNumber("ctrlr out", controllerOut);
  volts = std::clamp(controllerOut, -maxVolts, maxVolts);
  //m_motor.SetVoltage((units::volt_t)volts);

}

//if already stowed, wont do anything, otherwise will initiate stowing
void Climb::Stow() {
  // if (m_state == State::STOWED) return;
  ChangeState(State::STOWING);
  m_regPID.SetSetpoint(ClimbConstants::STOWED_POS);
}

//same function as the last one but extending instead of stowing
void Climb::Extend() {
  // if (m_state == State::EXTENDED) return;
  ChangeState(State::EXTENDING);
  m_regPID.SetSetpoint(ClimbConstants::EXTENDED_POS);
}

void Climb::Lift() {
  m_FFPID.SetSetpoint(m_currentPos, ClimbConstants::LIFTED_POS);
  ChangeState(State::LIFTING);
}

// changes the state and resets PIDS but only if requested state is actually a change
void Climb::ChangeState(Climb::State newState) {
  if (m_state == newState) return;
  m_state = newState;
  ResetPIDs();
}

Climb::State Climb::GetState() {
  return m_state; 
}

std::string Climb::StateToString(){
  switch(m_state){
    case  STOWED:
    return "stowed";
    case EXTENDED:
    return "extended";
    case STOWING:
    return "stowing"; 
    case EXTENDING:
    return "extending"; 
    case LIFTING:
    return "lifting";
  }
  return "";
}

void Climb::ZeroEncoder(){
  m_motor.SetSelectedSensorPosition(0);
}

void Climb::ResetPIDs(){
  m_regPID.Reset();
  if (ClimbConstants::FEEDFORWARD){
  }else {
    m_climbPID.Reset();  
  }
}