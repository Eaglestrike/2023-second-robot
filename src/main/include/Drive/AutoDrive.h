#pragma once

#include <memory>

#include "Drive/Odometry.h"
#include "Drive/SwerveControl.h"
#include "Util/thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Class for autonomous driving operations
*/
class AutoDrive {
public:
  struct FFConfig {
    double maxSpeed;
    double maxAccel;
  };

  struct Times {
    double startT;
    double maxSpeedT;
    double descentT;
    double endT;
  };

  enum ExecuteState {
    NOT_EXECUTING,
    EXECUTING_PATH,
    EXECUTING_TARGET,
  };

  AutoDrive(Odometry *m_odometry);

  // void SetAutoPath();
  // void StartPath();
  void SetAbsTargetPose(vec::Vector2D target, double ang);
  void SetRelTargetPose(vec::Vector2D delta, double ang);
  void SetFFPos(FFConfig ffPos);
  void SetFFAng(FFConfig ffAng);
  void StartPosMove();
  void StartAngMove();
  void StopPos();
  void StopAng();
  void Periodic();
  void SetPosPID(double kP, double kI, double kD);
  void SetAngPID(double kP, double kI, double kD);

  vec::Vector2D GetVel() const;
  double GetAngVel() const;

  ExecuteState GetPosExecuteState() const;
  ExecuteState GetAngExecuteState() const;

private:
  Odometry *m_odometry;

  double m_prevTime;

  vec::Vector2D m_curExpectedPos;
  double m_curExpectedAng;
  vec::Vector2D m_targetPos;
  double m_targetAng;

  vec::Vector2D m_curVel;
  double m_curAngVel;

  Times m_posTimes;
  Times m_angTimes;
  vec::Vector2D m_posVecDir;
  double m_angVecDir;
  FFConfig m_ffPos;
  FFConfig m_ffAng;

  ExecuteState m_posState;
  ExecuteState m_angState;

  vec::Vector2D m_prevPosErr;
  double m_prevAngErr;

  vec::Vector2D m_totalPosErr;
  double m_totalAngErr;

  double m_kPPos;
  double m_kIPos;
  double m_kDPos;

  double m_kPAng;
  double m_kIAng;
  double m_kDAng;

  void CalcTimes(FFConfig &config, double dist, Times &times);
  double GetSpeed(FFConfig &config, Times &times);

  vec::Vector2D GetPIDTrans(double deltaT);
  double GetPIDAng(double deltaT);

  // TEMP
  // double m_dist;
};