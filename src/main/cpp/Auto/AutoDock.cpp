#include "Auto/AutoDock.h"

#include <algorithm>
#include <cmath>

#include "Drive/DriveConstants.h"

/**
 * Constructor
 * 
 * @param isRed If robot is on red side
*/
AutoDock::AutoDock(bool isRed)
  : m_curState{NOT_DOCKING}, m_isRed{isRed}, m_kTilt{AutoConstants::KTILT}, m_preDockSpeed{AutoConstants::PRE_DOCK_SPEED},
  m_maxDockSpeed{AutoConstants::MAX_DOCK_SPEED}, m_preDockAng{AutoConstants::PRE_DOCK_ANG},
  m_dockAng{AutoConstants::DOCK_ANG}, m_dockedTol{AutoConstants::DOCKED_TOL}, m_roll{0}, m_pitch{0}, m_yaw{0} {}

/**
 * Gets current state
 * 
 * @returns current state
*/
AutoDock::State AutoDock::GetState() const {
  return m_curState;
}

/**
 * Gets current wheel velocity
 * 
 * @returns wheel velocity
*/
vec::Vector2D AutoDock::GetVel() const {
  if (m_curState == NOT_DOCKING || m_curState == DOCKED) {
    return {0, 0};
  }

  return m_outputVel;
}

/**
 * Updates odometry
 * 
 * @param r Roll in rad
 * @param p pitch in rad
 * @param y Yaw in rad
 * 
 * @note parameters should account for roll and pitch offsets (based on how navx is mounted)
*/
void AutoDock::UpdateOdom(double r, double p, double y) {
  m_roll = r;
  m_pitch = p;
  m_yaw = y;
}

/**
 * Sets tilt constant of porportionality
 * 
 * @param kTilt tilt constant
*/
void AutoDock::SetkTilt(double kTilt) {
  m_kTilt = kTilt;
}

/**
 * Sets pre-dock approach speed
 * 
 * @param preDockSpeed pre-dock approach speed 
*/
void AutoDock::SetPreDockSpeed(double preDockSpeed) {
  m_preDockSpeed = preDockSpeed;
}

/**
 * Sets maximum docking speed
 * 
 * This speed is used for the robot to actually get from ground fully
 * onto charge station
 * 
 * @param maxDockSpeed maximum docking sped
*/
void AutoDock::SetMaxDockSpeed(double maxDockSpeed) {
  m_maxDockSpeed = maxDockSpeed;
}

/**
 * Starts docking
*/
void AutoDock::Start() {
  if (m_curState != NOT_DOCKING) {
    return;
  }

  m_curState = PRE_DOCK;
}

/**
 * Resets docking state
*/
void AutoDock::Reset() {
  m_curState = NOT_DOCKING;
}

/**
 * Sets pre-dock angle
 * 
 * This is angle where robot is touching and lined up against charge station,
 * ready to go max docking speed to get onto chaarge station
 * 
 * @param preDockAng pre-dock angle
*/
void AutoDock::SetPreDockAng(double preDockAng) {
  m_preDockAng = preDockAng;
}

/**
 * Sets dock angle
 * 
 * This is the angle where robot is fully on charge station
 * 
 * @param dockAng Dock angle
*/
void AutoDock::SetDockAng(double dockAng) {
  m_dockAng = dockAng;
}

/**
 * Sets dock tolerance
 * 
 * This is the angle below which robot is considered fully docked
 * and can lock wheels
 * 
 * @param dockedTol
*/
void AutoDock::SetDockedTol(double dockedTol) {
  m_dockedTol = dockedTol;
}

/**
 * Sets side
 * 
 * @param isRed true if robot on red side, false if on blue
*/
void AutoDock::SetSide(bool isRed) {
  m_isRed = isRed;
}

/**
 * Periodic function
*/
void AutoDock::Periodic() {
  double tilt = GetTilt();
  double sign = m_isRed ? 1 : -1;

  switch (m_curState)
  {
  case NOT_DOCKING:
    m_outputVel = {0, 0};

    break; 
  case PRE_DOCK:
    if (std::abs(tilt) > m_preDockAng) {
      m_curState = TOUCH_STN;
    }

    m_outputVel = {sign * m_preDockSpeed, 0};

    break;
  case TOUCH_STN:
    if (std::abs(tilt) > m_dockAng) {
      m_curState = ON_STN;
    }

    m_outputVel = {sign * m_maxDockSpeed, 0};

    break;
  case ON_STN:
    if (std::abs(tilt) < m_dockedTol) {
      m_curState = DOCKED;
    }

    m_outputVel = {sign * m_kTilt * std::abs(tilt)};

    break;
  case DOCKED:
    m_outputVel = {0, 0};
    break;
  default:
    break;
  }
}

/**
 * Gets tilt from roll, pitch, yaw using Caleb method
 * 
 * @returns Tilt
*/
double AutoDock::GetTilt() const {
  // Credit: https://www.reddit.com/r/askmath/comments/107g72d/tilt_angle_when_pitch_roll_is_known/ 
  // double x = -std::sin(m_yaw) * std::cos(m_roll);
  // double y = std::cos(m_yaw) * std::cos(m_pitch) - std::sin(m_yaw) * std::sin(m_pitch) * std::sin(m_roll);
  // double z = std::cos(m_yaw) * std::sin(m_pitch) + std::sin(m_yaw) * std::cos(m_pitch) * std::sin(m_roll);

  // return std::asin(std::clamp(z, -1.0, 1.0));

  return m_pitch * std::sin(m_yaw) - m_roll * std::cos(m_yaw);
}

bool AutoDock::HasStarted() const {
  return m_curState != NOT_DOCKING;
}