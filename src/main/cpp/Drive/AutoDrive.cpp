#include "Drive/AutoDrive.h"

#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Util/Mathutil.h"

AutoDrive::AutoDrive(Odometry *odometry)
  : m_odometry{odometry}, m_targetAng{0}, m_curAng{0}, m_posTimes{0, 0, 0, 0},
  m_angTimes{0, 0, 0, 0}, m_angVecDir{0}, m_ffPos{0, 0}, m_ffAng{0, 0}, m_state{NOT_EXECUTING} {}


/**
 * Sets absolute target pose for robot
 * 
 * @param target Taraget position, in world coordinates
 * @param ang Target orientation, in radians
*/
void AutoDrive::SetAbsTargetPose(vec::Vector2D target, double ang) {
  if (m_state != NOT_EXECUTING) {
    return;
  }

  m_targetPos = target;
  m_targetAng = Utils::NormalizeAng(ang);
}

/**
 * Sets relative target pose for robot
 * 
 * @param target Taraget position relative to current position
 * @param ang Target orientation relative to current oreintaion, in radians
*/
void AutoDrive::SetRelTargetPose(vec::Vector2D delta, double ang) {
  if (m_state != NOT_EXECUTING) {
    return;
  }

  vec::Vector2D curPos = m_odometry->GetPosition();
  double curAng = m_odometry->GetAng();

  m_targetPos = curPos + delta;
  m_targetAng = Utils::NormalizeAng(curAng + ang);
}

/**
 * Sets feed forward parameters for translational motion
 * 
 * @param ffPos Feed forward parameters
*/
void AutoDrive::SetFFPos(FFConfig ffPos) {
  m_ffPos.maxAccel = ffPos.maxAccel;
  m_ffPos.maxSpeed = ffPos.maxSpeed;
}

/**
 * Sets feed forward parameters for angular motion
 * 
 * @param ffAng Feed fowrad parameters
*/
void AutoDrive::SetFFAng(FFConfig ffAng) {
  m_ffAng.maxAccel = ffAng.maxAccel;
  m_ffAng.maxSpeed = ffAng.maxSpeed;
}

/**
 * Starts executing the move both in the translational and rotational motion
*/
void AutoDrive::StartMove() {
  if (m_state == EXECUTING_TARGET) {
    return;
  }

  vec::Vector2D curPos = m_odometry->GetPosition();
  vec::Vector2D posDiff = m_targetPos - curPos;
  double curAng = m_odometry->GetAng();

  // calculates angular distance
  double dist;
  if (std::abs(m_targetAng - curAng) < 180) {
    if (curAng < m_targetAng) {
      m_angVecDir = 1;
    } else {
      m_angVecDir = -1;
    }
    dist = std::abs(m_targetAng - curAng);
  } else {
    if (curAng < m_targetAng) {
      m_angVecDir = -1;
    } else {
      m_angVecDir = 1;
    }
    dist = 2 * M_PI - std::abs(m_targetAng - curAng);
  }

  StartMove(m_ffPos, magn(posDiff), m_posTimes);  
  StartMove(m_ffAng, dist, m_angTimes);

  if (Utils::NearZero(posDiff)) {
    m_posVecDir = {1, 0};
  } else {
    m_posVecDir = normalize(posDiff);
  }

  m_state = EXECUTING_TARGET;
}

/**
 * Starts executing the move in one of translational/rotational
 * 
 * @param config The ff config parameters
 * @param dist The distance to move
 * @param times The tiemes parameters
*/
void AutoDrive::StartMove(FFConfig &config, double dist, Times &times) {
  double curT = Utils::GetCurTimeS();

  if (Utils::NearZero(config.maxAccel) || Utils::NearZero(config.maxSpeed)) {
    return;
  }

  // time to accelerate/decelerate
  double increaseT = config.maxSpeed / config.maxAccel;
  // time to maintain max speed
  double maintainT = 0;
  if (increaseT * config.maxSpeed > dist) {
    // check math, may be wrong
    increaseT = std::sqrt(dist / config.maxAccel);
    maintainT = 0;
  } else {
    maintainT = (dist - increaseT * config.maxSpeed) / config.maxSpeed;
  }

  times.startT = curT;
  times.maxSpeedT = curT + increaseT;
  times.descentT = curT + increaseT + maintainT;
  times.endT = curT + increaseT * 2 + maintainT;
}

/**
 * Stops executing command
*/
void AutoDrive::StopCmd() {
  m_state = NOT_EXECUTING;
}

/**
 * Gets current execute state
 * 
 * @returns Current execute state
*/
AutoDrive::ExecuteState AutoDrive::GetExecuteState() const {
  return m_state;
}

/**
 * Periodic function
*/
void AutoDrive::Periodic() { 
  frc::SmartDashboard::PutNumber("m state", m_state);
  switch (m_state) {
    case NOT_EXECUTING:
      m_curVel = {0, 0};
      m_curAng = 0;
      m_dist = 0;
      break;
    case EXECUTING_PATH:
      // TEMP
      m_curVel = {0, 0};
      m_curAng = 0;
      break;
    case EXECUTING_TARGET:
    {
      double speed = GetSpeed(m_ffPos, m_posTimes);
      frc::SmartDashboard::PutString("taarget pos", m_targetPos.toString());
      frc::SmartDashboard::PutNumber("cur speed", speed);
      frc::SmartDashboard::PutString("vec dir", m_posVecDir.toString());
      frc::SmartDashboard::PutNumber("cur dist", m_dist);
      m_dist += speed * 0.02;

      // double angSpeed = GetSpeed(m_ffAng, m_angTimes);

      if (speed >= 0) {
        m_curVel = m_posVecDir * speed;
      } else {
        m_curVel = {0, 0};
        m_state = NOT_EXECUTING;
      }

      // if (angSpeed >= 0) {
      //   m_curAng = m_angVecDir * angSpeed;
      // } else {
      //   // TODO have separate state mahcine for angle
      //   m_curAng = 0;
      // }
      break;
    }
  }
}

/**
 * Gets current speed in one of rotational/translational motion
 * 
 * @param config One of rotational/translational ff paraameters
 * @param times The times parameters for rotational/translational
 * 
 * @returns Speed
*/
double AutoDrive::GetSpeed(FFConfig &config, Times &times) {
  double curT = Utils::GetCurTimeS();
  if (curT < times.startT) {
    // shouldn't be here
    return -1;
  }
  if (curT > times.endT) {
    StopCmd();
    return -1;
  }

  double speed;
  if (curT < times.maxSpeedT) {
    speed = config.maxAccel * (curT - times.startT);
  } else if (curT < times.descentT) {
    speed = config.maxSpeed;
  } else {
    speed = config.maxAccel * (times.endT - curT);
  }

  return speed;
}

/**
 * Gets current translational velocity
 * 
 * @returns Translational velocity
*/
vec::Vector2D AutoDrive::GetVel() const {
  return m_curVel;
}

/**
 * Gets current rotational velocity
 * 
 * @returns Rotational velocity
*/
double AutoDrive::GetAngVel() const {
  return m_curAng;
}