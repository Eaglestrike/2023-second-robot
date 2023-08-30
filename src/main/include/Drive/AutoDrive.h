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

  enum ExecuteState {
    NOT_EXECUTING,
    EXECUTING_PATH,
    EXECUTING_TARGET
  };

  AutoDrive(SwerveControl *swerveController, Odometry *m_odometry);

  // void SetAutoPath();
  void SetAbsTargetPose(vec::Vector2D target, double ang);
  void SetRelTargetPose(vec::Vector2D deltaX, double ang);
  void SetFFPos(FFConfig ffPos);
  void SetFFAng(FFConfig ffAng);
  void StartPath();
  void StartMove();
  void StopCmd();
  void Periodic();  

  ExecuteState GetExecuteState();

private:
  SwerveControl *m_swerveController;
  Odometry *m_odometry;

  vec::Vector2D m_targetPos;
  double m_targetAng;

  double m_startTime;
  FFConfig m_ffPos;
  FFConfig m_ffAng;

  ExecuteState m_state;
};