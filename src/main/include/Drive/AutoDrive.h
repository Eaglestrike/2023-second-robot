#pragma once

#include <memory>

#include "SwerveControl.h"

/**
 * Class for autonomous driving operations
*/
class AutoDrive {
public:
  AutoDrive(SwerveControl *swerveController);

  void SetPeriodic();
  void SetTargetPos();
  void ExecuteCmd();
  void StopCmd();
  void Periodic();  

private:
  SwerveControl *m_swerveController;
};