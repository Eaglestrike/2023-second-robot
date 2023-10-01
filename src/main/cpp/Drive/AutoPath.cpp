#include "Drive/AutoPath.h"

AutoPath::AutoPath() : 
  m_curAng{0}, m_curAngVel{0}, m_prevTime{0}, m_calc{100}, m_curState{NOT_EXECUTING},
  m_kPPos{0}, m_kIPos{0}, m_kDPos{0} {}

void AutoPath::AddPose(SwervePose pose) {
  Pose2 hmPose = ConvPose(pose);
  m_calc.insertOrReplace(hmPose);
  m_angExecuteTimes[pose.time * 100] = pose.ang; 
}

void AutoPath::AddPoses(std::vector<SwervePose> poses) {
  for (auto pose : poses) {
    AddPose(pose);
  }
}

void AutoPath::ResetPath() {
  m_calc = Hermite2{100};
  m_angExecuteTimes.clear();
}

void AutoPath::SetPosPID(double kP, double kI, double kD) {
  m_kPPos = kP;
  m_kIPos = kI;
  m_kDPos = kD;
}

void AutoPath::SetAngPID(double kP, double kI, double kD) {
  m_autoLineup.SetAngPID(kP, kI, kD);
}

void AutoPath::SetFFAng(AutoLineup::FFConfig ffAng) {
  m_autoLineup.SetFFAng(ffAng);
}

void AutoPath::Stop() {
  m_curState = NOT_EXECUTING;
}

void AutoPath::StartMove() {
  m_curState = EXECUTING_PATH;
  m_startTime = Utils::GetCurTimeS();
  m_expectFinish = m_calc.getHighestTime();
}

void AutoPath::UpdateOdom(vec::Vector2D curPos, double curAng) {
  m_curPos = curPos;
  m_curAng = curAng;
}

void AutoPath::Periodic() {
  double curTime = Utils::GetCurTimeS();
  double deltaT = curTime - m_prevTime;

  switch (m_curState) {
    case NOT_EXECUTING:
      m_curVel = {0, 0};
      break;
    case EXECUTING_PATH:
    {
      double relTime = curTime - m_startTime;
      double highestTime = m_calc.getHighestTime();
      double getTime = (relTime <= highestTime ? relTime : highestTime);

      vec::Vector2D curExpectedVel, curExpectedPos;
      curExpectedVel = m_calc.getVel(getTime);
      curExpectedPos = m_calc.getPos(getTime);

      vec::Vector2D correctionVel = GetPIDTrans(deltaT, curExpectedPos);
      vec::Vector2D totalVel = curExpectedVel + correctionVel;

      if (relTime >= highestTime && Utils::NearZero(totalVel) && m_autoLineup.GetAngExecuteState() == AutoLineup::NOT_EXECUTING) {
        m_curState = NOT_EXECUTING; 
        m_curVel = {0, 0};
      } else {
        m_curVel = totalVel;
      }

      // TODO impelemnt angle
      break;
    }
  }

  m_prevTime = curTime;
}

AutoPath::Pose2 AutoPath::ConvPose(SwervePose pose) {
  return Pose2{pose.time, {pose.x, pose.y}, {pose.vx, pose.vy}};
}

vec::Vector2D AutoPath::GetPIDTrans(double deltaT, vec::Vector2D curExpectedPos) {
  vec::Vector2D err = curExpectedPos - m_curPos;

  vec::Vector2D deltaErr = (err - m_prevPosErr) / deltaT;
  m_totalPosErr += err * deltaT;
  vec::Vector2D res = err * m_kPPos + m_totalPosErr * m_kIPos + deltaErr * m_kDPos;

  m_prevPosErr = err;

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
