#pragma once

#include <cstddef>
#include <map>
#include <vector>

#include "Drive/AutoLineup.h"
#include "Util/Mathutil.h"
#include "Util/thirdparty/hermite.hpp"
#include "Util/thirdparty/simplevectors.hpp"

namespace hm = hermite;

class AutoPath {
  typedef hm::Hermite<2> Hermite2;
  typedef hm::Pose<2> Pose2;

public:
  struct SwervePose {
    double time;
    double x, y;
    double vx, vy;
    double ang;
  };

  enum ExecuteState {
    NOT_EXECUTING,
    EXECUTING_PATH
  };

  AutoPath();

  void AddPose(SwervePose pose);
  void AddPoses(std::vector<SwervePose> poses);
  void ResetPath();
  void SetPosPID(double kP, double kI, double kD);
  void SetAngPID(double kP, double kI, double kD);
  void Stop();
  void StartMove();
  void UpdateOdom(vec::Vector2D curPos, double curAng);
  void Periodic();

  vec::Vector2D GetVel() const;
  double GetAngVel() const;
  ExecuteState GetExecuteState() const;

private:
  vec::Vector2D m_curPos;
  double m_curAng;

  vec::Vector2D m_curVel;
  double m_curAngVel;

  double m_startTime;
  double m_expectFinish;
  double m_prevTime;

  Hermite2 m_calc;
  std::map<std::size_t, double> m_angExecuteTimes;
  AutoLineup m_autoLineup;

  ExecuteState m_curState;

  // for position PID
  vec::Vector2D m_prevPosErr;
  vec::Vector2D m_totalPosErr;

  double m_kPPos;
  double m_kIPos;
  double m_kDPos;

  Pose2 ConvPose(SwervePose pose);
  vec::Vector2D GetPIDTrans(double deltaT, vec::Vector2D curExpectedPos);
};