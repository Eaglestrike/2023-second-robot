#pragma once

#include <array>
#include <frc/controller/PIDController.h>
#include <functional>

#include "SwerveModule.h"
#include "Util/thirdparty/simplevectors.hpp"

#include "Util/Mechanism.h"

namespace vec = svector; //!< Alias to vector namespace

/**
 * Swerve controller for 4-wheel swerve
*/
class SwerveControl{
public:
  template <typename T>
  using RefArray = std::array<std::reference_wrapper<T>, 4>; //!< Alias to an array of references

  SwerveControl(RefArray<SwerveModule> modules, double kS, double kV, double kA);

  vec::Vector2D GetRobotVelocity(double ang);

  void ResetAngleCorrection();
  void ResetFeedForward();
  void SetFeedForward(double kS, double kV, double kA);
  void SetAngleCorrectionPID(double kP, double kI, double kD);
  void SetRobotVelocity(vec::Vector2D vel, double angVel, double ang, double time);

  void Periodic();

private:
  RefArray<SwerveModule> m_modules;
  std::array<double, 4> m_prevSpeeds;

  double m_kS;
  double m_kV;
  double m_kA;

  double m_curAngle;
  frc2::PIDController m_angleCorrector;
};
