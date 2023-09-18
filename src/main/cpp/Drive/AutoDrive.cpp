#include "Drive/AutoDrive.h"

#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433
#endif

#include "Util/Mathutil.h"

AutoDrive::AutoDrive(Odometry *odometry)
  : m_odometry{odometry}, m_prevTime{0}, m_curExpectedAng{0}, m_targetAng{0}, m_curAngVel{0}, m_posTimes{0, 0, 0, 0},
  m_angTimes{0, 0, 0, 0}, m_angVecDir{0}, m_ffPos{0, 0}, m_ffAng{0, 0}, m_posState{NOT_EXECUTING},
  m_prevPosErr{0}, m_prevAngErr{0}, m_totalPosErr{0}, m_totalAngErr{0},
  m_kPPos{0}, m_kIPos{0}, m_kDPos{0}, m_kPAng{0}, m_kIAng{0}, m_kDAng{0} {}


/**
 * Sets absolute target pose for robot
 * 
 * @param target Taraget position, in world coordinates
 * @param ang Target orientation, in radians
*/
void AutoDrive::SetAbsTargetPose(vec::Vector2D target, double ang) {
  if (m_posState != NOT_EXECUTING) {
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
  if (m_posState != NOT_EXECUTING) {
    return;
  }

  vec::Vector2D curPos = m_odometry->GetPosition();
  double curAng = m_odometry->GetAng();

  frc::SmartDashboard::PutString("Cur Pos", curPos.toString());

  m_targetPos = curPos + delta;
  m_targetAng = Utils::NormalizeAng(curAng + ang);
}

/**
 * Sets feed forward parameters for translational motion
 * 
 * Does not set if currently executing a path
 * 
 * @param ffPos Feed forward parameters
*/
void AutoDrive::SetFFPos(FFConfig ffPos) {
  if (m_posState == NOT_EXECUTING) {
    m_ffPos.maxAccel = ffPos.maxAccel;
    m_ffPos.maxSpeed = ffPos.maxSpeed;
  }
}

/**
 * Sets feed forward parameters for angular motion
 * 
 * Does not set if currently executing a path
 * 
 * @param ffAng Feed fowrad parameters
*/
void AutoDrive::SetFFAng(FFConfig ffAng) {
  if (m_angState == NOT_EXECUTING) {
    m_ffAng.maxAccel = ffAng.maxAccel;
    m_ffAng.maxSpeed = ffAng.maxSpeed;
  }
}

/**
 * Starts executing the move both in translational motion
*/
void AutoDrive::StartPosMove() {
  if (m_posState != NOT_EXECUTING) {
    return;
  }

  vec::Vector2D curPos = m_odometry->GetPosition();
  vec::Vector2D posDiff = m_targetPos - curPos;

  CalcTimes(m_ffPos, magn(posDiff), m_posTimes);  

  if (Utils::NearZero(posDiff)) {
    m_posVecDir = {1, 0};
  } else {
    m_posVecDir = normalize(posDiff);
  }

  m_curExpectedPos = curPos;
  m_prevPosErr = {0, 0};
  m_totalPosErr = {0, 0};
  m_posState = EXECUTING_TARGET;
}

/**
 * Start execuitng move in angular motion
*/
void AutoDrive::StartAngMove() {
  if (m_angState != NOT_EXECUTING) {
    return;
  }  
  double curAng = m_odometry->GetAng();

  // calculates angular distance
  double dist;
  if (std::abs(m_targetAng - curAng) < M_PI) {
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

  CalcTimes(m_ffAng, dist, m_angTimes);
  
  m_curExpectedAng = curAng;
  m_prevAngErr = 0;
  m_totalAngErr = 0;
  m_angState = EXECUTING_TARGET;
}

/**
 * Starts executing the move in one of translational/rotational
 * 
 * @param config The ff config parameters
 * @param dist The distance to move
 * @param times The tiemes parameters
*/
void AutoDrive::CalcTimes(FFConfig &config, double dist, Times &times) {
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
 * Stops executing position command
*/
void AutoDrive::StopPos() {
  m_posState = NOT_EXECUTING;
}

/**
 * Stops executing angle command
*/
void AutoDrive::StopAng() {
  m_angState = NOT_EXECUTING;
}

/**
 * Gets current position execute state
 * 
 * @returns Current execute state
*/
AutoDrive::ExecuteState AutoDrive::GetPosExecuteState() const {
  return m_posState;
}

/**
 * Get current ang execute state
 * 
 * @returns Current ang execute state
*/
AutoDrive::ExecuteState AutoDrive::GetAngExecuteState() const {
  return m_angState;
}

/**
 * Periodic function
*/
void AutoDrive::Periodic() { 
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  switch (m_posState) {
    case NOT_EXECUTING:
      m_curVel = {0, 0};
      break;
    case EXECUTING_PATH:
      // TEMP
      m_curVel = {0, 0};
      break;
    case EXECUTING_TARGET:
    {
      double speed = GetSpeed(m_ffPos, m_posTimes);
      vec::Vector2D ffVel = m_posVecDir * speed;
      m_curExpectedPos += ffVel * deltaT;

      vec::Vector2D correctionVel = GetPIDTrans(deltaT);
      vec::Vector2D totalVel = ffVel + correctionVel;

      // may need to change to checking setpoints
      if (!Utils::NearZero(totalVel)) {
        m_curVel = totalVel;
      } else {
        m_curVel = {0, 0};
        m_posState = NOT_EXECUTING;
      }

      break;
    }
  }

  // frc::SmartDashboard::PutNumber("m state", m_angState);
  switch (m_angState) {
    case NOT_EXECUTING:
      m_curAngVel = 0;
      // m_dist = 0;
      break;
    case EXECUTING_PATH:
      // TEMP
      m_curAngVel = 0;
      break;
    case EXECUTING_TARGET:
    {
      double angSpeed = GetSpeed(m_ffAng, m_angTimes);
      double ffAngVel = m_angVecDir * angSpeed;

      m_curExpectedAng += ffAngVel * (curTime - m_prevTime);
      m_curExpectedAng = Utils::NormalizeAng(m_curExpectedAng);

      double correctionVel = GetPIDAng(deltaT);
      double totalVel = ffAngVel + correctionVel;

      // frc::SmartDashboard::PutNumber("taarget ang", m_targetAng);
      // frc::SmartDashboard::PutNumber("cur speed", angSpeed);
      // frc::SmartDashboard::PutNumber("ang dir", m_angVecDir);
      // frc::SmartDashboard::PutNumber("cur dist", m_dist);
      // m_dist += angSpeed * 0.02;

      if (!Utils::NearZero(totalVel)) {
        m_curAngVel = totalVel;
      } else {
        m_curAngVel = 0;
        m_angState = NOT_EXECUTING;
      }
      break;
    }
  }

  m_prevTime = curTime;
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
    StopPos();
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
  return m_curAngVel;
}

/**
 * Calculates PID for trnaslational motion
 * 
 * @param deltaT time difference
 * 
 * @returns Velocity from PID
*/
vec::Vector2D AutoDrive::GetPIDTrans(double deltaT) {
  vec::Vector2D curPos = m_odometry->GetPosition();
  vec::Vector2D err = m_curExpectedPos - curPos;  

  vec::Vector2D deltaErr = (err - m_prevPosErr) / deltaT;
  m_totalPosErr += err * deltaT;
  vec::Vector2D res = err * m_kPPos + m_totalPosErr * m_kIPos + deltaErr * m_kDPos;

  m_prevPosErr = err;

  return res;
}

/**
 * Calculates PID for angular motion
 * 
 * @param deltaT time diffrence
 * 
 * @returns Velcoity from PID
*/
double AutoDrive::GetPIDAng(double deltaT) {
  double curAng = m_odometry->GetAng();

  double err;
  double dir;
  if (std::abs(m_curExpectedAng - curAng) < 180) {
    if (curAng < m_targetAng) {
      dir = 1;
    } else {
      dir = -1;
    }
    err = std::abs(m_curExpectedAng - curAng);
  } else {
    if (curAng < m_curExpectedAng) {
      dir = -1;
    } else {
      dir = 1;
    }
    err = 2 * M_PI - std::abs(m_curExpectedAng - curAng);
  }

  err = dir * err;

  double deltaErr = (err - m_prevAngErr) / deltaT;
  m_totalAngErr += err * deltaT;
  double res = err * m_kPAng + m_totalAngErr * m_kIAng + deltaErr * m_kDAng;

  m_prevAngErr = err;

  return res;
}

/**
 * Sets position correction PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void AutoDrive::SetPosPID(double kP, double kI, double kD) {
  m_kPPos = kP;
  m_kIPos = kI;
  m_kDPos = kD;
}

/**
 * Sets angle correction PID
 * 
 * @param kP p
 * @param kI i
 * @param kD d
*/
void AutoDrive::SetAngPID(double kP, double kI, double kD) {
  m_kPAng = kP;
  m_kIAng = kI;
  m_kDAng = kD;
}