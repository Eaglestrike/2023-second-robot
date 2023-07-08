#pragma once

#include <cstddef>
#include <memory>

#include <AHRS.h>

#include "KalmanFilter.h"
#include "SwerveControl.h"
#include "thirdparty/simplevectors.hpp"

namespace vec = svector;

/**
 * Gets information about robot's velocity, position, and heading
*/
class Odometry {
public:
  Odometry(vec::Vector2D *posOffset, double *angOffset);
  
  void SetKFTerms(double E0, double Q, double kAng, double k, double kPosInt, double maxTime);
  void SetCamData(vec::Vector2D camPos, double camAng, std::size_t tagID, std::size_t age, std::size_t uniqueId);
  void Reset();

  vec::Vector2D GetPosition() const;
  double GetAng() const;

  void Periodic(double ang, vec::Vector2D avgVelocity);

private:
  vec::Vector2D *m_posOffset;
  double *m_angOffset;
  KalmanFilter m_filter;
  long long m_prevId;
};