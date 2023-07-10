#include "Climb.h"

Climb::Climb() {
  m_regPID.SetTolerance(ClimbConstants::EXTND_STOW_POS_ERR_TOLERANCE,
                            ClimbConstants::EXTND_STOW_VEL_ERR_TOLERANCE);
  m_climbPID.SetTolerance(ClimbConstants::CLIMB_POS_ERR_TOLERANCE,
                            ClimbConstants::CLIMB_VEL_ERR_TOLERANCE);
}

void Climb::RobotInit(){
  m_state = State::STOWED;
  ZeroEncoder();
  ResetPIDs();
}

void Climb::updatePos(){
 double steps = m_motor.GetSelectedSensorPosition(); 
 double rad = steps * (2 * M_PI / 2048); // falcon is 2048 ticks per revolution
 m_currentPos = rad;
}

void Climb::Periodic() {
  updatePos();

  switch (m_state) {
    case State::STOWED:
    case State::EXTENDED:
      m_motor.SetVoltage((units::volt_t)0);
      break;
    case State::STOWING:
      double controllerOut = m_regPID.Calculate(m_currentPos, ClimbConstants::STOWED_POS);
      double volts = std::clamp(controllerOut, -ClimbConstants::EXTND_STOW_MAX_VOLTAGE, ClimbConstants::EXTND_STOW_MAX_VOLTAGE);
      m_motor.SetVoltage((units::volt_t)volts);
      if (m_regPID.AtSetpoint())
        m_state = STOWED;
      break;
    case State::EXTENDING:
      double controllerOut = m_regPID.Calculate(m_currentPos, ClimbConstants::EXTENDED_POS);
      double volts = std::clamp(controllerOut, -ClimbConstants::EXTND_STOW_MAX_VOLTAGE, ClimbConstants::EXTND_STOW_MAX_VOLTAGE);
      m_motor.SetVoltage((units::volt_t)volts);
      if (m_regPID.AtSetpoint())
        m_state = EXTENDED;
      break;
    case State::LIFTING:
      double controllerOut = m_climbPID.Calculate(m_currentPos, ClimbConstants::LIFTED_POS);
      double volts = std::clamp(controllerOut, -ClimbConstants::CLIMB_MAX_VOLTAGE, ClimbConstants::CLIMB_MAX_VOLTAGE);
      m_motor.SetVoltage((units::volt_t)volts);
      // if (m_PID.AtSetpoint())
      //   m_state = LIFTEDa;
      break;
  } 
}

void Climb::Stow() {
  if (m_state == State::STOWED) return;
  changeState(State::STOWING);
}

void Climb::Extend() {
  if (m_state == State::EXTENDED) return;
  changeState(State::EXTENDING);
}

void Climb::Lift() {
  // if (m_state == State::LIFTED) return;
  changeState(State::LIFTING);
}

void Climb::changeState(Climb::State newState) {
  if (m_state == newState) return;
  m_state = newState;
  ResetPIDs();
}

Climb::State Climb::getState() { return m_state; }

void Climb::ZeroEncoder(){
    m_motor.SetSelectedSensorPosition(0);
}

void Climb::ResetPIDs(){
  m_regPID.Reset();
  m_climbPID.Reset();
}