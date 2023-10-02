#include "Drive/AutoPath.h"

#include "Drive/DriveConstants.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399
#endif

AutoPath::AutoPath() : 
  m_curAng{0}, m_multiplier{0}, m_curAngVel{0}, m_prevTime{0}, m_prevTimeOdom{0},
  m_calcTrans{100}, m_calcAng{100}, m_curState{NOT_EXECUTING},
  m_kPPos{0}, m_kIPos{0}, m_kDPos{0} {}

void AutoPath::AddPose(SwervePose pose) {
  Pose2 poseTrans = {pose.time, {pose.x, pose.y}, {pose.vx, pose.vy}};
  Pose1 poseAng = {pose.time, {pose.ang}, {pose.angVel}};

  m_calcTrans.insertOrReplace(poseTrans);
  m_calcAng.insertOrReplace(poseAng);
}

void AutoPath::AddPoses(std::vector<SwervePose> poses) {
  for (auto pose : poses) {
    AddPose(pose);
  }
}

void AutoPath::ResetPath() {
  m_calcTrans = Hermite2{100};
  m_calcAng = Hermite1{100};
}

void AutoPath::SetPosPID(double kP, double kI, double kD) {
  m_kPPos = kP;
  m_kIPos = kI;
  m_kDPos = kD;
}

void AutoPath::SetAngPID(double kP, double kI, double kD) {
  m_kPAng = kP;
  m_kIAng = kI;
  m_kDAng = kD;
}

void AutoPath::Stop() {
  m_curState = NOT_EXECUTING;
}

void AutoPath::StartMove() {
  m_curState = EXECUTING_PATH;
  m_startTime = Utils::GetCurTimeS();
  m_expectFinish = m_calcTrans.getHighestTime();
}

void AutoPath::UpdateOdom(vec::Vector2D curPos, double curAng) {
  m_curPos = curPos;

  // update multiplier of angle
  double curTimeS = Utils::GetCurTimeS();
  double curAngSpeed = (curAng - m_curAng) / (curTimeS - m_prevTimeOdom);
  if (curAngSpeed < -AutoConstants::UNREASONABLE_ANG_SPEED) {
    // looping around counterclockwise
    m_multiplier++;
  } else if (curAngSpeed > AutoConstants::UNREASONABLE_ANG_SPEED) {
    // looping around clockwise
    m_multiplier--;
  }
  m_prevTimeOdom = curTimeS;

  m_curAng = curAng;
}

void AutoPath::Periodic() {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  switch (m_curState) {
    case NOT_EXECUTING:
      m_curVel = {0, 0};
      m_curAngVel = 0;
      break;
    case EXECUTING_PATH:
    {
      double relTime = curTime - m_startTime;
      double highestTime = m_calcTrans.getHighestTime();
      double getTime = (relTime <= highestTime ? relTime : highestTime);

      vec::Vector2D curExpectedPos = m_calcTrans.getPos(getTime);
      vec::Vector2D curExpectedVel = m_calcTrans.getVel(getTime);

      vec::Vector2D correctionVel = GetPIDTrans(deltaT, curExpectedPos);
      vec::Vector2D totalVel = curExpectedVel + correctionVel;

      double curExpectedAng = m_calcAng.getPos(getTime)[0];
      double curExpectedAngVel = m_calcAng.getVel(getTime)[0];

      double correctionAngVel = GetPIDAng(deltaT, curExpectedAng); 
      double totalAngVel = curExpectedAngVel + correctionAngVel;

      m_curVel = totalVel;
      m_curAngVel = totalAngVel;

      if (AtTarget()) {
        m_curState = AT_TARGET;
      }

      // TODO impelemnt angle
      break;
    }
    case AT_TARGET:
    {
      m_curVel = {0, 0};
      m_curAngVel = 0;

      if (!AtTarget()) {
        m_curState = NOT_EXECUTING;
      }

      break;
    }
  }

  m_prevTime = curTime;
}

bool AutoPath::AtTarget() const {
  return AtTarget(AutoConstants::POS_ERR_TOLERANCE, AutoConstants::VEL_ERR_TOLERANCE);
}

bool AutoPath::AtTarget(double posErrTol, double velErrTol) const {
  vec::Vector2D targetPos = m_calcTrans.getPos(m_calcTrans.getHighestTime());
  double targetAng = m_calcAng.getPos(m_calcAng.getHighestTime())[0];

  return Utils::NearZero(targetPos - m_curPos, posErrTol) && Utils::NearZero(m_curVel, velErrTol)
        && Utils::NearZero(targetAng - GetMultipliedAng(), posErrTol) && Utils::NearZero(m_curAngVel, velErrTol);
}

double AutoPath::GetMultipliedAng() const {
  return m_curAng + m_multiplier * M_PI * 2;
}

vec::Vector2D AutoPath::GetPIDTrans(double deltaT, vec::Vector2D curExpectedPos) {
  vec::Vector2D err = curExpectedPos - m_curPos;

  vec::Vector2D deltaErr = (err - m_prevPosErr) / deltaT;
  m_totalPosErr += err * deltaT;
  vec::Vector2D res = err * m_kPPos + m_totalPosErr * m_kIPos + deltaErr * m_kDPos;

  m_prevPosErr = err;

  return res;
}

double AutoPath::GetPIDAng(double deltaT, double curExpectedAng) {
  double curAngAbs = m_curAng + m_multiplier * M_PI * 2;
  double err = curExpectedAng - curAngAbs;

  double deltaErr = (err - m_prevAngErr) / deltaT;
  m_totalAngErr += err * deltaT;
  double res = err * m_kPAng + m_totalAngErr * m_kIPos + deltaErr * m_kDPos;

  m_prevAngErr = err;

  return res;
}

vec::Vector2D AutoPath::GetVel() const {
  return m_curVel;
}

double AutoPath::GetAngVel() const {
  // TODO add angle calculations
  return m_curAngVel;
}

AutoPath::ExecuteState AutoPath::GetExecuteState() const {
  return m_curState;
}

