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
    EXECUTING_TARGET
  };

  AutoDrive(Odometry *m_odometry);

  // void SetAutoPath();
  // void StartPath();
  void SetAbsTargetPose(vec::Vector2D target, double ang);
  void SetRelTargetPose(vec::Vector2D delta, double ang);
  void SetFFPos(FFConfig ffPos);
  void SetFFAng(FFConfig ffAng);
  void StartMove();
  void StopCmd();
  void Periodic();

  vec::Vector2D GetVel();
  double GetAngVel();
  vec::Vector2D GetExpectedPos();
  double GetExpectedAng();

  ExecuteState GetExecuteState() const;

private:
  Odometry *m_odometry;

  vec::Vector2D m_targetPos;
  double m_targetAng;

  vec::Vector2D m_curVel;
  double m_curAng;

  Times m_posTimes;
  Times m_angTimes;
  vec::Vector2D m_posVecDir;
  double m_angVecDir;
  FFConfig m_ffPos;
  FFConfig m_ffAng;

  ExecuteState m_state;

  void StartMove(FFConfig &config, double dist, Times &times);

  double GetSpeed(FFConfig &config, Times &times);
};