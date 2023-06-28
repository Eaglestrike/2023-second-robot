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
  Odometry();
  
  void SetKFTerms(double E0, double Q, double kAng, double k, double maxTime);
  void SetCamData(vec::Vector2D camPos, double camAng, double angNavX, std::size_t camID, std::size_t howLongAgo);
  void Reset();

  vec::Vector2D GetPosition(vec::Vector2D posOffset) const;
  double GetAng() const;

  void Periodic(double ang, vec::Vector2D avgVelocity);

private:
  KalmanFilter m_filter;
};